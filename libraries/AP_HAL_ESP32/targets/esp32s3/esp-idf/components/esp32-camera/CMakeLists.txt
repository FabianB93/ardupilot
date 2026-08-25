# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

set(priv_requires "")

# set conversion sources
set(srcs
  conversions/yuv.c
  conversions/to_jpg.cpp
  conversions/to_bmp.c
  conversions/jpge.cpp
  )

set(priv_include_dirs
  conversions/private_include
  )

set(include_dirs
  driver/include
  conversions/include
  )

# set driver sources only for supported platforms
if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s2" OR IDF_TARGET STREQUAL "esp32s3")
  list(APPEND srcs
    driver/esp_camera.c
    driver/esp_camera_af.c
    driver/cam_hal.c
    driver/sensor.c
    sensors/ov2640.c
    sensors/ov3660.c
    sensors/ov5640.c
    sensors/ov5640_af.c
    sensors/ov7725.c
    sensors/ov7670.c
    sensors/nt99141.c
    sensors/gc0308.c
    sensors/gc2145.c
    sensors/gc032a.c
    sensors/bf3005.c
    sensors/bf20a6.c
    sensors/sc101iot.c
    sensors/sc030iot.c
    sensors/sc031gs.c
    sensors/mega_ccm.c
    sensors/hm1055.c
    sensors/hm0360.c
    )

  list(APPEND priv_include_dirs
    driver/private_include
    sensors/private_include
    target/private_include
    )

  if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs
      target/xclk.c
      target/esp32/ll_cam.c
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND srcs
      target/xclk.c
      target/esp32s2/ll_cam.c
      )

    list(APPEND priv_include_dirs
      target/esp32s2/private_include
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND srcs
      target/esp32s3/ll_cam.c
      )
  endif()

  list(APPEND priv_requires
    freertos
    nvs_flash
    esp_mm
    esp_jpeg
  )

  set(min_version_for_esp_timer "4.2")
  if (idf_version VERSION_GREATER_EQUAL min_version_for_esp_timer)
    list(APPEND priv_requires esp_timer)
  endif()

  #
  # SCCB / I2C driver selection
  #
  # ArduPilot ESP32-S3 already uses ESP-IDF's new I2C master API
  # (i2c_new_master_bus, i2c_master_bus_add_device, ...).
  #
  # Therefore esp32-camera must use sccb-ng.c as well. Mixing sccb.c
  # (legacy driver/i2c.h) with the new I2C master driver causes the
  # ESP-IDF startup conflict check to abort.
  #
  # Our local sccb-ng.c is patched for ESP-IDF 5.3 and reuses the
  # ArduPilot-owned bus handle via ap_esp32_get_i2c_master_bus_handle().
  #
  if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND srcs
      driver/sccb-ng.c
    )

    list(APPEND priv_requires
      esp_driver_i2c
    )
  else()
    list(APPEND srcs
      driver/sccb.c
    )
  endif()

endif()

set(req driver)

if (idf_version VERSION_GREATER_EQUAL "6.0")
  list(APPEND priv_requires
    esp_driver_gpio
    esp_driver_spi
    esp_driver_i2c
  )

  list(APPEND req
    esp_driver_ledc
    esp_hal_clock
  )
endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS ${include_dirs}
  PRIV_INCLUDE_DIRS ${priv_include_dirs}
  REQUIRES ${req}
  PRIV_REQUIRES ${priv_requires}
)