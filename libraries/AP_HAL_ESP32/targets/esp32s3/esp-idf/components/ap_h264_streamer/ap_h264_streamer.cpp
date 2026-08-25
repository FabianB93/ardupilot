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

extern "C" {
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_sw.h"
}

static const char *TAG = "AP_H264";

namespace {

struct NALView {
    const uint8_t *data;
    size_t len;
};

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

class RTPH264Sender {
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

        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        if (sock_ < 0) {
            ESP_LOGE(TAG,
                     "video socket() failed: errno=%d internal=%u "
                     "largest_internal=%u dma=%u largest_dma=%u psram=%u",
                     errno,
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                     unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                     unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
            return false;
        }

        ESP_LOGI(TAG,
                 "[DBG] socket created: fd=%d internal=%u largest_internal=%u",
                 sock_,
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)));

        const int flags = fcntl(sock_, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        }

        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(port);
        if (inet_aton(ip, &dest_.sin_addr) == 0) {
            ESP_LOGE(TAG, "invalid QGC destination IP: %s", ip);
            close(sock_);
            sock_ = -1;
            return false;
        }

        ESP_LOGI(TAG, "RTP/H264 destination %s:%u", ip, unsigned(port));
        return true;
    }

    void end()
    {
        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }
    }

    bool send_frame(const uint8_t *data, size_t len)
    {
        NALView nals[24]{};
        const size_t nal_count = parse_annex_b(data, len, nals, sizeof(nals) / sizeof(nals[0]));
        if (nal_count == 0) {
            ESP_LOGW(TAG, "encoded frame contains no Annex-B NAL units");
            advance_timestamp();
            return false;
        }

        bool idr = false;
        for (size_t i = 0; i < nal_count; i++) {
            const uint8_t type = nals[i].data[0] & 0x1f;
            if (type == 5) {
                idr = true;
            } else if (type == 7) {
                save_parameter_set(sps_, sps_len_, sizeof(sps_), nals[i]);
            } else if (type == 8) {
                save_parameter_set(pps_, pps_len_, sizeof(pps_), nals[i]);
            }
        }

        bool ok = true;
        if (idr && sps_len_ && pps_len_) {
            ok &= send_nal(sps_, sps_len_, false);
            ok &= send_nal(pps_, pps_len_, false);
        }

        for (size_t i = 0; i < nal_count; i++) {
            const bool last_nal = (i + 1U == nal_count);
            ok &= send_nal(nals[i].data, nals[i].len, last_nal);
        }

        advance_timestamp();
        return ok;
    }

private:
    static bool is_start_code(const uint8_t *p, size_t remaining, size_t &size)
    {
        if (remaining >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1) {
            size = 3;
            return true;
        }
        if (remaining >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) {
            size = 4;
            return true;
        }
        return false;
    }

    static size_t parse_annex_b(const uint8_t *data, size_t len, NALView *out, size_t out_cap)
    {
        if (!data || !out || len < 4 || out_cap == 0) {
            return 0;
        }

        size_t count = 0;
        size_t pos = 0;
        while (pos < len && count < out_cap) {
            size_t sc_len = 0;
            while (pos < len && !is_start_code(data + pos, len - pos, sc_len)) {
                ++pos;
            }
            if (pos >= len) {
                break;
            }

            const size_t nal_start = pos + sc_len;
            size_t next = nal_start;
            size_t next_sc = 0;
            while (next < len && !is_start_code(data + next, len - next, next_sc)) {
                ++next;
            }

            if (next > nal_start) {
                out[count++] = {data + nal_start, next - nal_start};
            }
            pos = next;
        }
        return count;
    }

    static void save_parameter_set(uint8_t *dst, size_t &dst_len, size_t dst_cap, const NALView &nal)
    {
        if (nal.len <= dst_cap) {
            std::memcpy(dst, nal.data, nal.len);
            dst_len = nal.len;
        }
    }

    bool send_packet(const uint8_t *payload, size_t payload_len, bool marker)
    {
        uint8_t packet[AP_H264_RTP_MTU];
        if (payload_len + 12U > sizeof(packet)) {
            return false;
        }

        packet[0] = 0x80; // RTP v2
        packet[1] = uint8_t(AP_H264_RTP_PAYLOAD_TYPE & 0x7fU) | (marker ? 0x80U : 0U);
        packet[2] = uint8_t(sequence_ >> 8);
        packet[3] = uint8_t(sequence_ & 0xff);
        packet[4] = uint8_t(timestamp_ >> 24);
        packet[5] = uint8_t(timestamp_ >> 16);
        packet[6] = uint8_t(timestamp_ >> 8);
        packet[7] = uint8_t(timestamp_);
        packet[8] = uint8_t(AP_H264_RTP_SSRC >> 24);
        packet[9] = uint8_t(AP_H264_RTP_SSRC >> 16);
        packet[10] = uint8_t(AP_H264_RTP_SSRC >> 8);
        packet[11] = uint8_t(AP_H264_RTP_SSRC);
        std::memcpy(packet + 12, payload, payload_len);

        const ssize_t sent = sendto(sock_, packet, payload_len + 12U, 0,
                                    reinterpret_cast<const sockaddr *>(&dest_), sizeof(dest_));
        ++sequence_;

        if (sent < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENETUNREACH) {
                ESP_LOGD(TAG, "video sendto failed: errno=%d", errno);
            }
            return false;
        }
        return static_cast<size_t>(sent) == payload_len + 12U;
    }

    bool send_nal(const uint8_t *data, size_t len, bool last_nal_of_frame)
    {
        if (!data || len == 0) {
            return false;
        }

        constexpr size_t RTP_HEADER = 12;
        constexpr size_t FU_HEADERS = 2;
        constexpr size_t SINGLE_MAX = AP_H264_RTP_MTU - RTP_HEADER;
        constexpr size_t FU_MAX = AP_H264_RTP_MTU - RTP_HEADER - FU_HEADERS;

        if (len <= SINGLE_MAX) {
            return send_packet(data, len, last_nal_of_frame);
        }

        const uint8_t nal_header = data[0];
        const uint8_t nal_type = nal_header & 0x1fU;
        const uint8_t fu_indicator = (nal_header & 0xe0U) | 28U;

        size_t pos = 1;
        bool first = true;
        bool ok = true;
        while (pos < len) {
            const size_t frag = std::min(FU_MAX, len - pos);
            const bool last_fragment = (pos + frag == len);

            uint8_t payload[FU_MAX + FU_HEADERS];
            payload[0] = fu_indicator;
            payload[1] = nal_type |
                         (first ? 0x80U : 0U) |
                         (last_fragment ? 0x40U : 0U);
            std::memcpy(payload + 2, data + pos, frag);

            ok &= send_packet(payload, frag + 2U, last_nal_of_frame && last_fragment);
            pos += frag;
            first = false;
        }
        return ok;
    }

    void advance_timestamp()
    {
        timestamp_ += AP_H264_RTP_CLOCK_HZ / AP_H264_FPS;
    }

    int sock_ = -1;
    sockaddr_in dest_{};
    uint16_t sequence_ = 0;
    uint32_t timestamp_ = 0;
    uint8_t sps_[256]{};
    size_t sps_len_ = 0;
    uint8_t pps_[128]{};
    size_t pps_len_ = 0;
};

bool init_camera()
{
    ESP_LOGI(TAG, "[DBG] init_camera(): entered");

    /*
     * ArduPilot already owns I2C1 on GPIO39/38. Setting pin_sccb_sda=-1 makes
     * esp32-camera use that existing IDF I2C port via sccb_i2c_port instead of
     * calling i2c_driver_install() a second time.
     */
    camera_config_t cfg{};

    ESP_LOGI(TAG, "[DBG] camera config: SCCB port=%d XCLK=%dHz FB=PSRAM format=YUV422 size=QVGA", AP_H264_CAM_SCCB_I2C_PORT, AP_H264_CAM_XCLK_HZ);
    cfg.pin_pwdn = AP_H264_CAM_PIN_PWDN;
    cfg.pin_reset = AP_H264_CAM_PIN_RESET;
    cfg.pin_xclk = AP_H264_CAM_PIN_XCLK;
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
    cfg.pixel_format = PIXFORMAT_YUV422;
    cfg.frame_size = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 0;
    cfg.fb_count = 1;
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.sccb_i2c_port = AP_H264_CAM_SCCB_I2C_PORT;

    ESP_LOGI(TAG, "[DBG] calling esp_camera_init()");
    const esp_err_t err = esp_camera_init(&cfg);
    ESP_LOGI(TAG, "[DBG] esp_camera_init() returned: 0x%x", int(err));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", int(err));
        return false;
    }

    ESP_LOGI(TAG, "[DBG] querying camera sensor handle");
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        /* Keep orientation unchanged by default; change here if mechanically required. */
        sensor->set_vflip(sensor, 1);
        sensor->set_hmirror(sensor, 1);
    }

    ESP_LOGI(TAG, "[DBG] camera sensor setup complete");
    ESP_LOGI(TAG, "OV2640 initialized: %ux%u YUV422", AP_H264_WIDTH, AP_H264_HEIGHT);
    return true;
}

} // namespace

extern "C" void ap_h264_streamer_run(void)
{
    ESP_LOGI(TAG, "[DBG] H264 task entered");
    ESP_LOGI(TAG, "[DBG] core=%d stack_hwm=%u", xPortGetCoreID(), unsigned(uxTaskGetStackHighWaterMark(nullptr)));
    ESP_LOGI(TAG, "[DBG] checking PSRAM");
    const size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (free_psram == 0) {
        ESP_LOGE(TAG, "No PSRAM available; H.264 streaming disabled");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "PSRAM available: %u bytes", unsigned(free_psram));
    ESP_LOGI(TAG, "[DBG] internal heap free: %u bytes", unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));

    /*
     * Memory-sensitive initialization order:
     *
     * 1. Initialize camera first, because its DMA setup needs the remaining
     *    DMA-capable internal RAM.
     * 2. Create the UDP/RTP socket immediately afterwards, while there is still
     *    enough normal internal RAM for lwIP.
     * 3. Only then create the software H.264 encoder, which consumes nearly all
     *    remaining internal RAM but can use PSRAM for its large working memory.
     */
    RTPH264Sender rtp;

    ESP_LOGI(TAG, "[DBG] starting camera init");
    log_heap_state("before camera init");
    if (!init_camera()) {
        ESP_LOGE(TAG, "[DBG] camera init failed");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "[DBG] camera init finished successfully");
    log_heap_state("after camera init");

    ESP_LOGI(TAG, "[DBG] opening RTP socket after camera init to %s:%u",
             AP_H264_DEST_IP, AP_H264_RTP_PORT);
    log_heap_state("before RTP socket after camera");
    if (!rtp.begin(AP_H264_DEST_IP, AP_H264_RTP_PORT)) {
        ESP_LOGE(TAG, "[DBG] RTP socket creation after camera failed");
        esp_camera_deinit();
        vTaskDelete(nullptr);
        return;
    }
    log_heap_state("after RTP socket after camera");

    ESP_LOGI(TAG, "[DBG] preparing H264 encoder config");

    esp_h264_enc_cfg_sw_t enc_cfg{};
    enc_cfg.pic_type = ESP_H264_RAW_FMT_YUYV;
    enc_cfg.gop = AP_H264_GOP;
    enc_cfg.fps = AP_H264_FPS;
    enc_cfg.res.width = AP_H264_WIDTH;
    enc_cfg.res.height = AP_H264_HEIGHT;
    enc_cfg.rc.bitrate = AP_H264_BITRATE;
    enc_cfg.rc.qp_min = AP_H264_QP_MIN;
    enc_cfg.rc.qp_max = AP_H264_QP_MAX;

    esp_h264_enc_handle_t encoder = nullptr;
    log_heap_state("before esp_h264_enc_sw_new");
    ESP_LOGI(TAG, "[DBG] calling esp_h264_enc_sw_new()");
    esp_h264_err_t hret = esp_h264_enc_sw_new(&enc_cfg, &encoder);
    ESP_LOGI(TAG, "[DBG] esp_h264_enc_sw_new() returned: %d encoder=%p", int(hret), encoder);
    log_heap_state("after esp_h264_enc_sw_new");
    if (hret != ESP_H264_ERR_OK || encoder == nullptr) {
        ESP_LOGE(TAG, "esp_h264_enc_sw_new failed: %d", int(hret));
        esp_camera_deinit();
        rtp.end();
        vTaskDelete(nullptr);
        return;
    }

    log_heap_state("before esp_h264_enc_open");
    ESP_LOGI(TAG, "[DBG] calling esp_h264_enc_open()");
    hret = esp_h264_enc_open(encoder);
    ESP_LOGI(TAG, "[DBG] esp_h264_enc_open() returned: %d", int(hret));
    log_heap_state("after esp_h264_enc_open");
    if (hret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "esp_h264_enc_open failed: %d", int(hret));
        esp_h264_enc_del(encoder);
        esp_camera_deinit();
        rtp.end();
        vTaskDelete(nullptr);
        return;
    }

    log_heap_state("before H264 output buffer");
    ESP_LOGI(TAG, "[DBG] allocating H264 output buffer: %u bytes in PSRAM", unsigned(AP_H264_OUTPUT_BUFFER_SIZE));
    uint8_t *out_buf = static_cast<uint8_t *>(
        heap_caps_malloc(AP_H264_OUTPUT_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!out_buf) {
        ESP_LOGE(TAG, "cannot allocate %u byte H264 output buffer in PSRAM",
                 unsigned(AP_H264_OUTPUT_BUFFER_SIZE));
        esp_h264_enc_close(encoder);
        esp_h264_enc_del(encoder);
        esp_camera_deinit();
        rtp.end();
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "[DBG] H264 output buffer allocated at %p", out_buf);
    log_heap_state("after H264 output buffer");

    /*
     * RTP socket was already created after camera initialization and before
     * software H.264 encoder creation. Do not create another socket here.
     */

    ESP_LOGI(TAG, "H264 streamer active: %ux%u @ %u fps, %u bit/s, RTP/UDP %s:%u",
             AP_H264_WIDTH, AP_H264_HEIGHT, AP_H264_FPS,
             unsigned(AP_H264_BITRATE), AP_H264_DEST_IP, AP_H264_RTP_PORT);

    ESP_LOGI(TAG, "[DBG] entering capture/encode loop");
    const TickType_t frame_period = pdMS_TO_TICKS(1000U / AP_H264_FPS);
    TickType_t next_frame = xTaskGetTickCount();
    uint32_t pts = 0;
    uint32_t frame_counter = 0;

    while (true) {
        if (frame_counter == 0U) {
            ESP_LOGI(TAG, "[DBG] requesting first camera frame");
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (frame_counter == 0U && fb != nullptr) {
            ESP_LOGI(TAG, "[DBG] first frame received: len=%u format=%d buf=%p", unsigned(fb->len), int(fb->format), fb->buf);
        }
        if (!fb) {
            ESP_LOGW(TAG, "camera frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const size_t expected = size_t(AP_H264_WIDTH) * AP_H264_HEIGHT * 2U;
        if (fb->format != PIXFORMAT_YUV422 || fb->len < expected) {
            ESP_LOGW(TAG, "unexpected camera frame: format=%d len=%u expected>=%u",
                     int(fb->format), unsigned(fb->len), unsigned(expected));
            esp_camera_fb_return(fb);
            vTaskDelayUntil(&next_frame, frame_period);
            continue;
        }

        esp_h264_enc_in_frame_t in{};
        esp_h264_enc_out_frame_t out{};
        in.raw_data.buffer = fb->buf;
        in.raw_data.len = fb->len;
        in.pts = pts++;
        out.raw_data.buffer = out_buf;
        out.raw_data.len = AP_H264_OUTPUT_BUFFER_SIZE;

        if (frame_counter == 0U) {
            ESP_LOGI(TAG, "[DBG] encoding first frame");
        }
        hret = esp_h264_enc_process(encoder, &in, &out);
        if (frame_counter == 0U) {
            ESP_LOGI(TAG, "[DBG] first encode returned: ret=%d len=%u", int(hret), unsigned(out.length));
        }
        if (hret == ESP_H264_ERR_OK && out.length > 0) {
            (void)rtp.send_frame(out.raw_data.buffer, out.length);
        } else {
            ESP_LOGW(TAG, "H264 encode failed: ret=%d len=%u", int(hret), unsigned(out.length));
        }

        esp_camera_fb_return(fb);

        if ((++frame_counter % 50U) == 0U) {
            ESP_LOGI(TAG, "video alive: frame=%u free_psram=%u free_internal=%u stack_hwm=%u",
                     unsigned(frame_counter),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
                     unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                     unsigned(uxTaskGetStackHighWaterMark(nullptr)));
        }

        /* The video task is intentionally low priority. If encoding took longer
         * than one frame period, vTaskDelayUntil() returns immediately and the
         * scheduler still allows all higher-priority ArduPilot work to run first.
         */
        vTaskDelayUntil(&next_frame, frame_period);
    }
}
