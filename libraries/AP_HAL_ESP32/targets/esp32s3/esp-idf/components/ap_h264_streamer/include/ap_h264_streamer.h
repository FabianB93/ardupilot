#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Run the H.264/RTP camera streamer. This function does not return during
 * normal operation and is intended to be called from the low-priority
 * ArduPilot video task.
 */
void ap_h264_streamer_run(void);

#ifdef __cplusplus
}
#endif
