#pragma once

/*
 * Native ESP-IDF / ArduPilot configuration shim.
 *
 * The upstream Arduino port included Arduino.h and relied on Arduino-specific
 * target detection. In the ArduPilot ESP-IDF build we explicitly define
 * HAVE_ESP32S3=1 from the component CMakeLists.txt.
 */

#if !defined(HAVE_ESP32S3)
#error "The bundled software H.264 encoder is only supported on ESP32-S3"
#endif