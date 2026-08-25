/*
 * ESP32 Camera SCCB driver using the ESP-IDF new I2C master API.
 *
 * ArduPilot ESP32-S3 / ESP-IDF 5.3 adaptation:
 * - reuses an ArduPilot-owned I2C master bus
 * - does not use i2c_master_get_bus_handle()
 * - never deletes an ArduPilot-owned bus
 * - synchronizes SCCB transactions with ArduPilot's per-bus semaphore
 * - probes only the explicitly requested SCCB address
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sccb.h"

static const char *TAG = "sccb-ng";

#define SCCB_FREQ               100000
#define SCCB_I2C_PORT_DEFAULT   I2C_NUM_0
#define SCCB_TIMEOUT_MS         100
#define SCCB_LOCK_TIMEOUT_MS    100
#define MAX_DEVICES             16
#define MAX_ERROR_LOGS          10

typedef struct {
    uint8_t addr;
    i2c_master_dev_handle_t dev_handle;
} device_t;

static device_t devices[MAX_DEVICES];
static uint8_t device_count = 0;
static int sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
static bool sccb_owns_i2c_port = false;
static i2c_master_bus_handle_t sccb_bus_handle = NULL;
static uint32_t sccb_error_logs = 0;

/*
 * Implemented in AP_HAL_ESP32/I2CDevice.cpp.
 */
extern i2c_master_bus_handle_t
ap_esp32_get_i2c_master_bus_handle(int i2c_port);

extern bool
ap_esp32_lock_i2c_master_bus(int i2c_port, uint32_t timeout_ms);

extern void
ap_esp32_unlock_i2c_master_bus(int i2c_port);

static bool lock_bus(void)
{
    /*
     * A bus created by SCCB_Init() is private to this driver.
     * Only ArduPilot-owned buses need the ArduPilot semaphore.
     */
    if (sccb_owns_i2c_port) {
        return true;
    }

    if (!ap_esp32_lock_i2c_master_bus(sccb_i2c_port,
                                      SCCB_LOCK_TIMEOUT_MS)) {
        if (sccb_error_logs < MAX_ERROR_LOGS) {
            ESP_LOGW(TAG,
                     "Could not acquire ArduPilot I2C%d semaphore",
                     sccb_i2c_port);
            sccb_error_logs++;
        }
        return false;
    }

    return true;
}

static void unlock_bus(void)
{
    if (!sccb_owns_i2c_port) {
        ap_esp32_unlock_i2c_master_bus(sccb_i2c_port);
    }
}

static void log_i2c_error(const char *op,
                          uint8_t slv_addr,
                          esp_err_t err)
{
    if (sccb_error_logs < MAX_ERROR_LOGS) {
        ESP_LOGW(TAG,
                 "%s failed addr=0x%02x: %s",
                 op,
                 slv_addr,
                 esp_err_to_name(err));
        sccb_error_logs++;

        if (sccb_error_logs == MAX_ERROR_LOGS) {
            ESP_LOGW(TAG,
                     "Further SCCB transaction errors will be suppressed");
        }
    }
}

static i2c_master_dev_handle_t find_device(uint8_t slv_addr)
{
    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].addr == slv_addr) {
            return devices[i].dev_handle;
        }
    }

    return NULL;
}

/*
 * Must be called while the corresponding bus lock is held when using an
 * ArduPilot-owned bus.
 */
static esp_err_t install_device_locked(uint8_t slv_addr,
                                       i2c_master_dev_handle_t *dev_handle)
{
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sccb_bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_dev_handle_t existing = find_device(slv_addr);
    if (existing != NULL) {
        *dev_handle = existing;
        return ESP_OK;
    }

    if (device_count >= MAX_DEVICES) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slv_addr,
        .scl_speed_hz = SCCB_FREQ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false,
    };

    esp_err_t ret = i2c_master_bus_add_device(
        sccb_bus_handle,
        &dev_cfg,
        &devices[device_count].dev_handle);

    if (ret != ESP_OK) {
        return ret;
    }

    devices[device_count].addr = slv_addr;
    *dev_handle = devices[device_count].dev_handle;
    device_count++;

    return ESP_OK;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    sccb_bus_handle = NULL;
    device_count = 0;
    sccb_error_logs = 0;
    memset(devices, 0, sizeof(devices));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = SCCB_I2C_PORT_DEFAULT,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    const esp_err_t ret =
        i2c_new_master_bus(&bus_cfg, &sccb_bus_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "Failed to create dedicated SCCB bus: %s",
                 esp_err_to_name(ret));
        sccb_bus_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG,
             "Created dedicated SCCB bus port=%d SDA=%d SCL=%d",
             SCCB_I2C_PORT_DEFAULT,
             pin_sda,
             pin_scl);

    return ESP_OK;
}

int SCCB_Use_Port(int i2c_num)
{
    if (i2c_num < 0 || i2c_num >= I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sccb_owns_i2c_port && sccb_bus_handle != NULL) {
        const int ret = SCCB_Deinit();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    sccb_i2c_port = i2c_num;
    sccb_owns_i2c_port = false;
    device_count = 0;
    sccb_error_logs = 0;
    memset(devices, 0, sizeof(devices));

    sccb_bus_handle =
        ap_esp32_get_i2c_master_bus_handle(i2c_num);

    if (sccb_bus_handle == NULL) {
        ESP_LOGE(TAG,
                 "ArduPilot I2C bus %d is not initialized",
                 i2c_num);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "Using ArduPilot I2C bus %d, handle=%p",
             i2c_num,
             sccb_bus_handle);

    return ESP_OK;
}

int SCCB_Deinit(void)
{
    esp_err_t first_error = ESP_OK;

    if (!lock_bus()) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].dev_handle == NULL) {
            continue;
        }

        const esp_err_t ret =
            i2c_master_bus_rm_device(devices[i].dev_handle);

        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }

        devices[i].dev_handle = NULL;
    }

    unlock_bus();

    device_count = 0;
    memset(devices, 0, sizeof(devices));

    /*
     * An ArduPilot-owned bus is only detached, never deleted.
     */
    if (!sccb_owns_i2c_port) {
        sccb_bus_handle = NULL;
        return first_error;
    }

    if (sccb_bus_handle != NULL) {
        const esp_err_t ret =
            i2c_del_master_bus(sccb_bus_handle);

        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
    }

    sccb_bus_handle = NULL;
    sccb_owns_i2c_port = false;

    return first_error;
}

int SCCB_Probe(uint8_t slv_addr)
{
    if (sccb_bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!lock_bus()) {
        return ESP_ERR_TIMEOUT;
    }

    const esp_err_t ret =
        i2c_master_probe(sccb_bus_handle,
                         slv_addr,
                         SCCB_TIMEOUT_MS);

    unlock_bus();

    if (ret == ESP_OK) {
        ESP_LOGI(TAG,
                 "SCCB device acknowledged at 0x%02x",
                 slv_addr);
    }

    /*
     * A NACK during sensor detection is expected for addresses that are not
     * populated, so do not emit an additional SCCB warning here.
     */
    return ret;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    if (!lock_bus()) {
        return 0;
    }

    i2c_master_dev_handle_t dev_handle = NULL;

    esp_err_t ret =
        install_device_locked(slv_addr, &dev_handle);

    uint8_t value = 0;

    if (ret == ESP_OK) {
        ret = i2c_master_transmit_receive(
            dev_handle,
            &reg,
            1,
            &value,
            1,
            SCCB_TIMEOUT_MS);
    }

    unlock_bus();

    if (ret != ESP_OK) {
        log_i2c_error("Read8", slv_addr, ret);
        return 0;
    }

    return value;
}

int SCCB_Write(uint8_t slv_addr,
               uint8_t reg,
               uint8_t data)
{
    if (!lock_bus()) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle = NULL;

    esp_err_t ret =
        install_device_locked(slv_addr, &dev_handle);

    if (ret == ESP_OK) {
        const uint8_t payload[2] = {
            reg,
            data
        };

        ret = i2c_master_transmit(
            dev_handle,
            payload,
            sizeof(payload),
            SCCB_TIMEOUT_MS);
    }

    unlock_bus();

    if (ret != ESP_OK) {
        log_i2c_error("Write8", slv_addr, ret);
    }

    return ret;
}

uint8_t SCCB_Read16(uint8_t slv_addr,
                    uint16_t reg)
{
    if (!lock_bus()) {
        return 0;
    }

    i2c_master_dev_handle_t dev_handle = NULL;

    esp_err_t ret =
        install_device_locked(slv_addr, &dev_handle);

    const uint8_t reg_buf[2] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xff)
    };

    uint8_t value = 0;

    if (ret == ESP_OK) {
        ret = i2c_master_transmit_receive(
            dev_handle,
            reg_buf,
            sizeof(reg_buf),
            &value,
            1,
            SCCB_TIMEOUT_MS);
    }

    unlock_bus();

    if (ret != ESP_OK) {
        log_i2c_error("Read16", slv_addr, ret);
        return 0;
    }

    return value;
}

int SCCB_Write16(uint8_t slv_addr,
                 uint16_t reg,
                 uint8_t data)
{
    if (!lock_bus()) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle = NULL;

    esp_err_t ret =
        install_device_locked(slv_addr, &dev_handle);

    if (ret == ESP_OK) {
        const uint8_t payload[3] = {
            (uint8_t)(reg >> 8),
            (uint8_t)(reg & 0xff),
            data
        };

        ret = i2c_master_transmit(
            dev_handle,
            payload,
            sizeof(payload),
            SCCB_TIMEOUT_MS);
    }

    unlock_bus();

    if (ret != ESP_OK) {
        log_i2c_error("Write16", slv_addr, ret);
    }

    return ret;
}
