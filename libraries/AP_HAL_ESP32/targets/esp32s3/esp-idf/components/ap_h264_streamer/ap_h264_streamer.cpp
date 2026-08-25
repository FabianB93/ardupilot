#include "ap_h264_streamer.h"
#include "ap_h264_streamer_config.h"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

static const char *TAG = "AP_JPEG";

#ifndef AP_H264_JPEG_QUALITY
#define AP_H264_JPEG_QUALITY 12
#endif

#ifndef AP_H264_RTP_JPEG_PAYLOAD_TYPE
// RTP/AVP static payload type for JPEG (RFC 3551).
#define AP_H264_RTP_JPEG_PAYLOAD_TYPE 26
#endif

namespace {

static void log_heap_state(const char *where)
{
    ESP_LOGI(TAG,
             "[MEM] %s: internal=%u largest_internal=%u "
             "dma=%u largest_dma=%u psram=%u",
             where,
             unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
             unsigned(heap_caps_get_free_size(MALLOC_CAP_DMA)),
             unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
             unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}

struct JPEGFrameInfo {
    const uint8_t *scan = nullptr;
    size_t scan_len = 0;

    uint16_t width = 0;
    uint16_t height = 0;

    // RFC 2435 type 0 = 4:2:2, type 1 = 4:2:0.
    uint8_t type = 0;

    uint8_t qtable_luma[64]{};
    uint8_t qtable_chroma[64]{};
    bool have_luma_qtable = false;
    bool have_chroma_qtable = false;

    uint16_t restart_interval = 0;
};

static bool marker_has_length(uint8_t marker)
{
    // Markers without a 16-bit segment length.
    if (marker == 0xD8 || marker == 0xD9 || marker == 0x01) {
        return false;
    }
    if (marker >= 0xD0 && marker <= 0xD7) {
        return false;
    }
    return true;
}

static bool parse_dqt(const uint8_t *data, size_t len, JPEGFrameInfo &info)
{
    size_t pos = 0;

    while (pos < len) {
        const uint8_t pq_tq = data[pos++];
        const uint8_t precision = pq_tq >> 4;
        const uint8_t table_id = pq_tq & 0x0F;

        // RFC 2435 quantization-table header below uses 8-bit tables.
        if (precision != 0) {
            ESP_LOGE(TAG, "16-bit JPEG quantization tables are not supported");
            return false;
        }

        if (pos + 64U > len) {
            return false;
        }

        if (table_id == 0) {
            std::memcpy(info.qtable_luma, data + pos, 64);
            info.have_luma_qtable = true;
        } else if (table_id == 1) {
            std::memcpy(info.qtable_chroma, data + pos, 64);
            info.have_chroma_qtable = true;
        }

        pos += 64U;
    }

    return true;
}

static bool parse_sof0(const uint8_t *data, size_t len, JPEGFrameInfo &info)
{
    // precision(1), height(2), width(2), components(1), then 3 bytes/component
    if (len < 6) {
        return false;
    }

    if (data[0] != 8) {
        ESP_LOGE(TAG, "JPEG precision %u not supported", unsigned(data[0]));
        return false;
    }

    info.height = (uint16_t(data[1]) << 8) | data[2];
    info.width = (uint16_t(data[3]) << 8) | data[4];

    const uint8_t components = data[5];
    if (components != 3 || len < size_t(6 + components * 3)) {
        ESP_LOGE(TAG, "RFC2435 requires 3-component baseline JPEG");
        return false;
    }

    uint8_t y_sampling = 0;
    uint8_t cb_sampling = 0;
    uint8_t cr_sampling = 0;

    for (uint8_t i = 0; i < components; i++) {
        const uint8_t component_id = data[6 + i * 3];
        const uint8_t sampling = data[7 + i * 3];

        if (component_id == 1) {
            y_sampling = sampling;
        } else if (component_id == 2) {
            cb_sampling = sampling;
        } else if (component_id == 3) {
            cr_sampling = sampling;
        }
    }

    if (y_sampling == 0x21 && cb_sampling == 0x11 && cr_sampling == 0x11) {
        info.type = 0; // 4:2:2
    } else if (y_sampling == 0x22 && cb_sampling == 0x11 && cr_sampling == 0x11) {
        info.type = 1; // 4:2:0
    } else {
        ESP_LOGE(TAG,
                 "unsupported JPEG sampling: Y=0x%02x Cb=0x%02x Cr=0x%02x",
                 y_sampling, cb_sampling, cr_sampling);
        return false;
    }

    return true;
}

static bool parse_jpeg_for_rtp(const uint8_t *jpeg, size_t jpeg_len, JPEGFrameInfo &info)
{
    if (jpeg == nullptr || jpeg_len < 4 ||
        jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
        ESP_LOGE(TAG, "invalid JPEG SOI");
        return false;
    }

    size_t pos = 2;

    while (pos + 1 < jpeg_len) {
        // Locate the next marker prefix.
        while (pos < jpeg_len && jpeg[pos] != 0xFF) {
            pos++;
        }
        if (pos >= jpeg_len) {
            break;
        }

        while (pos < jpeg_len && jpeg[pos] == 0xFF) {
            pos++;
        }
        if (pos >= jpeg_len) {
            break;
        }

        const uint8_t marker = jpeg[pos++];

        if (marker == 0xD9) {
            break;
        }

        if (!marker_has_length(marker)) {
            continue;
        }

        if (pos + 2 > jpeg_len) {
            return false;
        }

        const uint16_t seg_len = (uint16_t(jpeg[pos]) << 8) | jpeg[pos + 1];
        if (seg_len < 2) {
            return false;
        }

        pos += 2;
        const size_t payload_len = size_t(seg_len) - 2U;

        if (pos + payload_len > jpeg_len) {
            return false;
        }

        const uint8_t *payload = jpeg + pos;

        switch (marker) {
        case 0xDB: // DQT
            if (!parse_dqt(payload, payload_len, info)) {
                return false;
            }
            break;

        case 0xC0: // SOF0 - baseline DCT
            if (!parse_sof0(payload, payload_len, info)) {
                return false;
            }
            break;

        case 0xDD: // DRI
            if (payload_len != 2) {
                return false;
            }
            info.restart_interval =
                (uint16_t(payload[0]) << 8) | payload[1];
            break;

        case 0xDA: { // SOS
            // Entropy-coded scan begins immediately after the SOS segment.
            const size_t scan_start = pos + payload_len;
            if (scan_start >= jpeg_len) {
                return false;
            }

            size_t scan_end = jpeg_len;

            // Drop the final EOI marker from the RTP JPEG payload.
            if (jpeg_len >= 2 &&
                jpeg[jpeg_len - 2] == 0xFF &&
                jpeg[jpeg_len - 1] == 0xD9) {
                scan_end -= 2;
            }

            if (scan_end <= scan_start) {
                return false;
            }

            info.scan = jpeg + scan_start;
            info.scan_len = scan_end - scan_start;

            if (info.width == 0 || info.height == 0 ||
                !info.have_luma_qtable || !info.have_chroma_qtable) {
                ESP_LOGE(TAG,
                         "JPEG missing SOF0 or quantization tables "
                         "(w=%u h=%u q0=%d q1=%d)",
                         unsigned(info.width),
                         unsigned(info.height),
                         int(info.have_luma_qtable),
                         int(info.have_chroma_qtable));
                return false;
            }

            return true;
        }

        default:
            break;
        }

        pos += payload_len;
    }

    ESP_LOGE(TAG, "JPEG SOS marker not found");
    return false;
}

class RTPJPEG2435Sender {
public:
    bool begin(const char *ip, uint16_t port)
    {
        ESP_LOGI(TAG,
                 "[DBG] before socket: internal=%u largest_internal=%u "
                 "dma=%u largest_dma=%u psram=%u",
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                 unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));

        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ < 0) {
            ESP_LOGE(TAG, "JPEG RTP socket() failed: errno=%d", errno);
            return false;
        }

        const int flags = fcntl(sock_, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        }

        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(port);

        if (inet_aton(ip, &dest_.sin_addr) == 0) {
            ESP_LOGE(TAG, "invalid destination IP: %s", ip);
            end();
            return false;
        }

        ESP_LOGI(TAG, "RTP/JPEG RFC2435 destination %s:%u", ip, unsigned(port));
        return true;
    }

    void end()
    {
        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }
    }

    bool send_frame(const uint8_t *jpeg, size_t jpeg_len)
    {
        JPEGFrameInfo info{};
        if (!parse_jpeg_for_rtp(jpeg, jpeg_len, info)) {
            return false;
        }

        if ((info.width % 8U) != 0 || (info.height % 8U) != 0 ||
            info.width > 2040 || info.height > 2040) {
            ESP_LOGE(TAG, "RFC2435 unsupported dimensions %ux%u",
                     unsigned(info.width), unsigned(info.height));
            return false;
        }

        /*
         * RFC 2435 Q=255 means the quantization-table mapping is dynamic.
         * The first packet of every frame therefore carries both 64-byte
         * quantization tables exactly as extracted from the JPEG DQT markers.
         */
        constexpr uint8_t Q = 255;

        // Restart markers are supported by RFC 2435, but the OV2640 normally
        // emits JPEG without a DRI segment. Keep this implementation strict
        // until a camera stream with DRI is actually needed.
        if (info.restart_interval != 0) {
            ESP_LOGE(TAG,
                     "JPEG DRI=%u detected; RFC2435 restart-marker packetization "
                     "is not enabled yet",
                     unsigned(info.restart_interval));
            return false;
        }

        size_t offset = 0;

        while (offset < info.scan_len) {
            const bool first = (offset == 0);

            constexpr size_t RTP_HEADER_SIZE = 12;
            constexpr size_t JPEG_HEADER_SIZE = 8;
            constexpr size_t QTABLE_HEADER_SIZE = 4;
            constexpr size_t QTABLE_DATA_SIZE = 128;

            const size_t extra_first =
                first ? (QTABLE_HEADER_SIZE + QTABLE_DATA_SIZE) : 0U;

            const size_t headers =
                RTP_HEADER_SIZE + JPEG_HEADER_SIZE + extra_first;

            if (headers >= AP_H264_RTP_MTU) {
                return false;
            }

            const size_t max_payload = AP_H264_RTP_MTU - headers;
            const size_t chunk =
                std::min(max_payload, info.scan_len - offset);
            const bool last = (offset + chunk == info.scan_len);

            uint8_t packet[AP_H264_RTP_MTU]{};
            uint8_t *p = packet;

            // RTP header.
            p[0] = 0x80; // Version 2
            p[1] = uint8_t(AP_H264_RTP_JPEG_PAYLOAD_TYPE & 0x7FU) |
                   (last ? 0x80U : 0U);
            p[2] = uint8_t(sequence_ >> 8);
            p[3] = uint8_t(sequence_ & 0xFF);
            p[4] = uint8_t(timestamp_ >> 24);
            p[5] = uint8_t(timestamp_ >> 16);
            p[6] = uint8_t(timestamp_ >> 8);
            p[7] = uint8_t(timestamp_);
            p[8] = uint8_t(AP_H264_RTP_SSRC >> 24);
            p[9] = uint8_t(AP_H264_RTP_SSRC >> 16);
            p[10] = uint8_t(AP_H264_RTP_SSRC >> 8);
            p[11] = uint8_t(AP_H264_RTP_SSRC);
            p += RTP_HEADER_SIZE;

            // RFC 2435 main JPEG header.
            p[0] = 0; // Type-specific: progressive image
            p[1] = uint8_t((offset >> 16) & 0xFF);
            p[2] = uint8_t((offset >> 8) & 0xFF);
            p[3] = uint8_t(offset & 0xFF);
            p[4] = info.type;
            p[5] = Q;
            p[6] = uint8_t(info.width / 8U);
            p[7] = uint8_t(info.height / 8U);
            p += JPEG_HEADER_SIZE;

            if (first) {
                // RFC 2435 Quantization Table header.
                p[0] = 0; // MBZ
                p[1] = 0; // 8-bit precision for both tables
                p[2] = 0;
                p[3] = QTABLE_DATA_SIZE;
                p += QTABLE_HEADER_SIZE;

                std::memcpy(p, info.qtable_luma, 64);
                p += 64;
                std::memcpy(p, info.qtable_chroma, 64);
                p += 64;
            }

            std::memcpy(p, info.scan + offset, chunk);
            p += chunk;

            const size_t packet_len = size_t(p - packet);

            const ssize_t sent =
                sendto(sock_,
                       packet,
                       packet_len,
                       0,
                       reinterpret_cast<const sockaddr *>(&dest_),
                       sizeof(dest_));

            sequence_++;

            if (sent < 0) {
                if (errno != EAGAIN &&
                    errno != EWOULDBLOCK &&
                    errno != ENETUNREACH) {
                    ESP_LOGW(TAG,
                             "RTP/JPEG sendto failed: errno=%d",
                             errno);
                }
                return false;
            }

            if (size_t(sent) != packet_len) {
                return false;
            }

            offset += chunk;
        }

        // RFC 2435 uses the normal RTP 90 kHz clock for JPEG.
        timestamp_ += 90000U / AP_H264_FPS;
        return true;
    }

private:
    int sock_ = -1;
    sockaddr_in dest_{};
    uint16_t sequence_ = 0;
    uint32_t timestamp_ = 0;
};

bool init_camera()
{
    ESP_LOGI(TAG, "[DBG] init_camera(): entered");

    camera_config_t cfg{};

    cfg.pin_pwdn = AP_H264_CAM_PIN_PWDN;
    cfg.pin_reset = AP_H264_CAM_PIN_RESET;
    cfg.pin_xclk = AP_H264_CAM_PIN_XCLK;

    /*
     * ArduPilot already owns the selected I2C port. The adapted SCCB driver
     * reuses that bus when SDA/SCL are -1 and sccb_i2c_port is supplied.
     */
    cfg.pin_sccb_sda = -1;
    cfg.pin_sccb_scl = -1;

    cfg.pin_d7 = AP_H264_CAM_PIN_D7;
    cfg.pin_d6 = AP_H264_CAM_PIN_D6;
    cfg.pin_d5 = AP_H264_CAM_PIN_D5;
    cfg.pin_d4 = AP_H264_CAM_PIN_D4;
    cfg.pin_d3 = AP_H264_CAM_PIN_D3;
    cfg.pin_d2 = AP_H264_CAM_PIN_D2;
    cfg.pin_d1 = AP_H264_CAM_PIN_D1;
    cfg.pin_d0 = AP_H264_CAM_PIN_D0;

    cfg.pin_vsync = AP_H264_CAM_PIN_VSYNC;
    cfg.pin_href = AP_H264_CAM_PIN_HREF;
    cfg.pin_pclk = AP_H264_CAM_PIN_PCLK;

    cfg.xclk_freq_hz = AP_H264_CAM_XCLK_HZ;
    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;

    /*
     * Critical change from the H.264 path:
     * the OV2640 now performs JPEG compression itself.
     */
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size = FRAMESIZE_QVGA;
    cfg.jpeg_quality = AP_H264_JPEG_QUALITY;
    cfg.fb_count = 1;
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.sccb_i2c_port = AP_H264_CAM_SCCB_I2C_PORT;

    ESP_LOGI(TAG,
             "[DBG] camera config: JPEG QVGA quality=%u SCCB=%d XCLK=%dHz",
             unsigned(AP_H264_JPEG_QUALITY),
             AP_H264_CAM_SCCB_I2C_PORT,
             AP_H264_CAM_XCLK_HZ);

    const esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", int(err));
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != nullptr) {
        // Camera is physically mounted 180 degrees rotated.
        sensor->set_vflip(sensor, 1);
        sensor->set_hmirror(sensor, 1);
    }

    ESP_LOGI(TAG, "OV2640 initialized: JPEG QVGA quality=%u",
             unsigned(AP_H264_JPEG_QUALITY));
    return true;
}

} // namespace

extern "C" void ap_h264_streamer_run(void)
{
    /*
     * Function name intentionally retained for now so no Scheduler/HAL API
     * changes are required while migrating from H.264 to RFC2435 JPEG.
     */
    ESP_LOGI(TAG, "RFC2435 JPEG video task entered");
    ESP_LOGI(TAG,
             "core=%d stack_hwm=%u",
             xPortGetCoreID(),
             unsigned(uxTaskGetStackHighWaterMark(nullptr)));

    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "No PSRAM available; JPEG streaming disabled");
        vTaskDelete(nullptr);
        return;
    }

    log_heap_state("before camera init");

    if (!init_camera()) {
        vTaskDelete(nullptr);
        return;
    }

    log_heap_state("after camera init");

    RTPJPEG2435Sender rtp;
    if (!rtp.begin(AP_H264_DEST_IP, AP_H264_RTP_PORT)) {
        esp_camera_deinit();
        vTaskDelete(nullptr);
        return;
    }

    log_heap_state("after RTP socket");

    ESP_LOGI(TAG,
             "RFC2435 JPEG streamer active: QVGA @ %u fps, quality=%u, "
             "RTP/UDP %s:%u PT=%u",
             unsigned(AP_H264_FPS),
             unsigned(AP_H264_JPEG_QUALITY),
             AP_H264_DEST_IP,
             AP_H264_RTP_PORT,
             unsigned(AP_H264_RTP_JPEG_PAYLOAD_TYPE));

    const TickType_t frame_period =
        pdMS_TO_TICKS(std::max(1U, 1000U / unsigned(AP_H264_FPS)));

    TickType_t next_frame = xTaskGetTickCount();
    uint32_t frame_counter = 0;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();

        if (fb == nullptr) {
            ESP_LOGW(TAG, "camera frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (fb->format != PIXFORMAT_JPEG || fb->buf == nullptr || fb->len == 0) {
            ESP_LOGW(TAG,
                     "unexpected camera frame: format=%d len=%u",
                     int(fb->format),
                     unsigned(fb->len));
            esp_camera_fb_return(fb);
            vTaskDelayUntil(&next_frame, frame_period);
            continue;
        }

        const bool sent = rtp.send_frame(fb->buf, fb->len);

        if (!sent) {
            ESP_LOGW(TAG,
                     "RFC2435 frame send failed: frame=%u jpeg_len=%u",
                     unsigned(frame_counter),
                     unsigned(fb->len));
        }

        esp_camera_fb_return(fb);

        frame_counter++;

        if (frame_counter == 1U || (frame_counter % 50U) == 0U) {
            ESP_LOGI(TAG,
                     "JPEG video alive: frame=%u free_psram=%u "
                     "free_internal=%u stack_hwm=%u",
                     unsigned(frame_counter),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                     unsigned(uxTaskGetStackHighWaterMark(nullptr)));
        }

        vTaskDelayUntil(&next_frame, frame_period);
    }
}
