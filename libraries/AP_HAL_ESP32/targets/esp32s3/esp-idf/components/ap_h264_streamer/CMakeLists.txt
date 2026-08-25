set(H264_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/esp_h264")

set(H264_SRCS
    "${H264_DIR}/h264/esp_h264_alloc.c"
    "${H264_DIR}/h264/esp_h264_alloc_less_than_5_3.c"
    "${H264_DIR}/h264/esp_h264_cache.c"
    "${H264_DIR}/h264/esp_h264_enc_single.c"
    "${H264_DIR}/h264/esp_h264_enc_single_sw.c"
    "${H264_DIR}/h264/esp_h264_enc_sw_param.c"
    "${H264_DIR}/h264/h264_color_convert.c"
    "${H264_DIR}/h264/h264_color_convert.S"
)

idf_component_register(
    SRCS
        "ap_h264_streamer.cpp"
        "openh264_compat.c"
        ${H264_SRCS}

    INCLUDE_DIRS
        "include"
        "."
        "${H264_DIR}"
        "${H264_DIR}/h264"

    REQUIRES
        esp32-camera
        esp_jpeg
        lwip
        esp_psram
        esp_mm
)

target_compile_definitions(
    ${COMPONENT_LIB}
    PRIVATE
        HAVE_ESP32S3=1
)

target_link_libraries(
    ${COMPONENT_LIB}
    PRIVATE
        "${H264_DIR}/esp32s3/libopenh264.a"
)

target_link_options(
    ${COMPONENT_LIB}
    INTERFACE
        "-Wl,-u,sysconf"
)
