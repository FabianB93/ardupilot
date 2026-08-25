# AP ESP32-S3 H.264 streamer

Native ESP-IDF integration for the `esp32s3drone` ArduPilot target.

The encoder subset and `libopenh264.a` originate from the user-supplied
`pschatzmann/codec-h264-ESP32S3` project. Arduino wrappers (`.ino`, `Print`,
`WiFiUDP`, `Arduino.h`) are intentionally not used.

Pipeline:

`OV2640 YUV422 -> Espressif SW H.264/OpenH264 -> RFC 6184 RTP -> UDP/5600 -> QGroundControl`

The camera SCCB interface reuses ArduPilot I2C1 rather than installing another
I2C driver on GPIO39/38.
