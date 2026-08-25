#pragma once

/* esp32s3drone OV2640 pin mapping */
#define AP_H264_CAM_PIN_PWDN      (-1)
#define AP_H264_CAM_PIN_RESET     (-1)
#define AP_H264_CAM_PIN_XCLK      21
#define AP_H264_CAM_PIN_D7        47
#define AP_H264_CAM_PIN_D6        14
#define AP_H264_CAM_PIN_D5        13
#define AP_H264_CAM_PIN_D4        11
#define AP_H264_CAM_PIN_D3        9
#define AP_H264_CAM_PIN_D2        3
#define AP_H264_CAM_PIN_D1        46
#define AP_H264_CAM_PIN_D0        10
#define AP_H264_CAM_PIN_VSYNC     45
#define AP_H264_CAM_PIN_HREF      48
#define AP_H264_CAM_PIN_PCLK      12
#define AP_H264_CAM_XCLK_HZ       20000000

/*
 * SCCB shares ArduPilot I2C1 (GPIO39 SDA / GPIO38 SCL).
 * esp32-camera is told to use the already initialized bus instead of
 * reconfiguring the I2C peripheral behind ArduPilot's back.
 */
#define AP_H264_CAM_SCCB_I2C_PORT 1

/* Conservative first-flight video settings */
#define AP_H264_WIDTH             320
#define AP_H264_HEIGHT            240
#define AP_H264_FPS               10
#define AP_H264_GOP               5
#define AP_H264_BITRATE           250000U
#define AP_H264_QP_MIN            28
#define AP_H264_QP_MAX            35

/* RTP/H.264 settings for QGroundControl */
#define AP_H264_RTP_PORT          5600
#define AP_H264_RTP_PAYLOAD_TYPE  96
#define AP_H264_RTP_CLOCK_HZ      90000U
#define AP_H264_RTP_MTU           1200U
#define AP_H264_RTP_SSRC          0x41504452U /* 'APDR' */

/* Change this if QGC receives another DHCP address from the ESP32 AP. */
#define AP_H264_DEST_IP           "192.168.4.2"

/* QVGA H.264 output buffer; stored in PSRAM. */
#define AP_H264_OUTPUT_BUFFER_SIZE (256U * 1024U)

