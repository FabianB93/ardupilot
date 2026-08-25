#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: patch_esp32_camera_lowprio.py <esp32-camera-dir>")

root = Path(sys.argv[1])
cam_hal = root / "driver" / "cam_hal.c"
kconfig = root / "Kconfig"

if not cam_hal.exists() or not kconfig.exists():
    raise SystemExit("invalid esp32-camera directory")

text = cam_hal.read_text()
old = "configMAX_PRIORITIES - 2"
if old in text:
    text = text.replace(old, "CONFIG_CAMERA_TASK_PRIORITY")
    cam_hal.write_text(text)

ktext = kconfig.read_text()
marker = 'config CAMERA_TASK_PRIORITY\n'
if marker not in ktext:
    addition = '''\nconfig CAMERA_TASK_PRIORITY\n    int "Camera frame-processing task priority"\n    range 1 24\n    default 3\n    help\n        Low priority is intentional for ArduPilot: flight control, timer,\n        RC, UART and WiFi tasks must pre-empt camera frame processing.\n\n'''
    # Place before DMA buffer configuration if possible.
    needle = 'config CAMERA_DMA_BUFFER_SIZE_MAX\n'
    if needle in ktext:
        ktext = ktext.replace(needle, addition + needle, 1)
    else:
        ktext += addition
    kconfig.write_text(ktext)

print("patched:", cam_hal)
print("patched:", kconfig)
