# ESP32-S3 OV2640 -> H.264/RTP -> QGroundControl

This integration is native ESP-IDF/ArduPilot code. No Arduino `.ino`, `Print`,
`WiFiUDP`, `delay()` or `millis()` API is used.

## Architecture

- OV2640 captures `320x240 PIXFORMAT_YUV422`.
- The camera SCCB interface reuses ArduPilot `I2C1` (GPIO39/GPIO38) by setting
  `pin_sccb_sda = -1` and `sccb_i2c_port = 1`.
- Espressif's software H.264 encoder/OpenH264 (from the supplied
  `codec-h264-ESP32S3` archive) encodes the raw frame.
- `ap_h264_streamer` parses Annex-B NAL units and packetizes them as RFC 6184
  RTP with FU-A fragmentation.
- RTP payload type is 96, clock rate is 90 kHz, UDP destination is
  `192.168.4.2:5600`.
- QGroundControl must use `UDP h.264 Video Stream`, port `5600`.

## ArduPilot task priorities

The video path is deliberately below all flight-critical work:

- APM_MAIN: 24, core 0
- APM_TIMER/UART: 23
- WiFi: 20/12, core 1
- RCOUT: 10
- I2C/RCIN/IO: 5
- Storage: 4
- esp32-camera internal frame task: **3, core 1** (patched)
- APM_VIDEO/H.264/RTP: **2, core 1**

`APM_VIDEO` waits until ArduPilot has called `set_system_initialized()` before
initializing the camera. This is required because camera SCCB shares I2C1 with
the BME280.

## One-time camera component installation

The official camera driver is intentionally kept as a separate third-party IDF
component instead of copying its source into AP_HAL.

From the ArduPilot repository root run:

```bash
libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/components/install_esp32_camera.sh
```

The script downloads `espressif/esp32-camera v2.1.7` and patches its internal
`cam_task` priority. The upstream driver otherwise creates this task at
`configMAX_PRIORITIES - 2`, which is too high for a flight controller.

After installing/updating esp32-camera, run the patch script again if needed:

```bash
python3 libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/components/patch_esp32_camera_lowprio.py \
  libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/components/esp32-camera
```

## PSRAM is mandatory

The software encoder allocates substantial working memory and the camera frame
buffer/output buffer are placed in PSRAM. The runtime streamer refuses to start
if `esp_psram_is_initialized()` is false.

Enable PSRAM in the ESP-IDF sdkconfig according to the exact ESP32-S3-WROOM
module fitted to the PCB (octal/quad mode must match the hardware). Do not copy
an arbitrary PSRAM mode from another board.

For raw YUV camera capture, `CONFIG_CAMERA_PSRAM_DMA=y` is already enabled in
`sdkconfig.defaults`.

## Build

After installing esp32-camera and configuring PSRAM:

```bash
./waf configure --board=esp32s3drone --debug
./waf copter
```

If the previous `esp-idf_build` directory was configured before installing the
camera component, delete the board build directory or run a clean configure so
CMake enumerates the new component.

## QGroundControl

Connect the QGC device to the ESP32 access point (`ardupilot123`). Then select:

- Video Source: `UDP h.264 Video Stream`
- UDP Port: `5600`

The default target is `192.168.4.2`. If the QGC device receives another DHCP
address, change `AP_H264_DEST_IP` in:

`targets/esp32s3/esp-idf/components/ap_h264_streamer/ap_h264_streamer_config.h`

## Initial stream settings

- Resolution: 320x240 (QVGA)
- FPS: 5
- Bitrate: 400 kbit/s
- GOP: 5
- QP: 28..35
- RTP MTU: 1200 bytes
- H.264 output buffer: 256 KiB PSRAM
- APM_VIDEO stack: 32 KiB (monitor `stack_hwm` in the periodic video log)

These are intentionally conservative for running H.264 on the same ESP32-S3 as
ArduCopter. Increase only after checking scheduler timing, watchdog behaviour,
IMU timing and motor output stability.

## Camera pins

- XCLK: GPIO21
- SCCB SDA/SCL: GPIO39/GPIO38 via existing I2C1
- D7..D0: GPIO47, GPIO14, GPIO13, GPIO11, GPIO9, GPIO3, GPIO46, GPIO10
- VSYNC: GPIO45
- HREF: GPIO48
- PCLK: GPIO12

GPIO3, GPIO45 and GPIO46 are ESP32-S3 strapping-related pins. In this design
they are camera-to-MCU input signals, but the electrical levels during reset
must still remain compatible with the required boot strapping.
