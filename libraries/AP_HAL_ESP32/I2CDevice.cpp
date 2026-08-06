/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/Semaphores.h>
#include <AP_Math/AP_Math.h>

using namespace ESP32;

#define MHZ (1000U * 1000U)
#define KHZ (1000U)

I2CBusDesc i2c_bus_desc[] = { HAL_ESP32_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            businfo[i].sw_handle.sda = i2c_bus_desc[i].sda;
            businfo[i].sw_handle.scl = i2c_bus_desc[i].scl;
            // TODO make modular
            businfo[i].sw_handle.speed = I2C_SPEED_FAST;
            businfo[i].soft = true;
            i2c_init(&businfo[i].sw_handle);
            continue;
        }

        businfo[i].soft = false;
        businfo[i].bus_clock = i2c_bus_desc[i].speed;

        i2c_master_bus_config_t bus_config {};
        bus_config.i2c_port = i2c_bus_desc[i].port;
        bus_config.sda_io_num = static_cast<gpio_num_t>(i2c_bus_desc[i].sda);
        bus_config.scl_io_num = static_cast<gpio_num_t>(i2c_bus_desc[i].scl);
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.intr_priority = 0;
        bus_config.trans_queue_depth = 0;
        bus_config.flags.enable_internal_pullup = true;

        const esp_err_t result = i2c_new_master_bus(&bus_config, &businfo[i].bus_handle);
        if (result != ESP_OK) {
            businfo[i].bus_handle = nullptr;
            printf("I2C: failed to create master bus %u: %s\n",
                   static_cast<unsigned>(i),
                   esp_err_to_name(result));
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(2),
    _address(address),
    _timeout_ms(timeout_ms),
    _bus_clock(bus_clock)
{
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x", static_cast<unsigned>(busnum), static_cast<unsigned>(address));

    if (!bus.soft) {
        configure_device(_address);
    }
}


bool I2CDevice::configure_device(uint8_t address)
{
    if (bus.soft) {
        _address = address;
        return true;
    }

    if (bus.bus_handle == nullptr) {
        return false;
    }

    if (device_handle != nullptr) {
        const esp_err_t remove_result = i2c_master_bus_rm_device(device_handle);
        if (remove_result != ESP_OK) {
            return false;
        }
        device_handle = nullptr;
    }

    i2c_device_config_t device_config {};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = address;
    device_config.scl_speed_hz = _bus_clock != 0 ? _bus_clock : bus.bus_clock;
    device_config.scl_wait_us = 0;
    device_config.flags.disable_ack_check = false;

    const esp_err_t result = i2c_master_bus_add_device(bus.bus_handle, &device_config, &device_handle);
    if (result != ESP_OK) {
        device_handle = nullptr;
        return false;
    }

    _address = address;
    set_device_address(address);
    return true;
}

void I2CDevice::set_address(uint8_t address)
{
    if (address == _address) {
        return;
    }

    configure_device(address);
}

I2CDevice::~I2CDevice()
{
    if (device_handle != nullptr) {
        const esp_err_t result = i2c_master_bus_rm_device(device_handle);
        if (result != ESP_OK) {
            printf("I2C: failed to remove device 0x%02x: %s\n",
                   static_cast<unsigned>(_address),
                   esp_err_to_name(result));
        }
        device_handle = nullptr;
    }

    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        printf("I2C: not owner of 0x%x\n", static_cast<unsigned>(get_bus_id()));
        return false;
    }

    if ((send_len != 0 && send == nullptr) || (recv_len != 0 && recv == nullptr)) {
        return false;
    }

    if (send_len == 0 && recv_len == 0) {
        return true;
    }

    if (bus.soft) {
        const uint8_t flag_wr = recv_len == 0 ? I2C_NOSTOP : 0;

        if (send_len != 0) {
            i2c_write_bytes(&bus.sw_handle, _address, send, send_len, flag_wr);
        }

        if (recv_len != 0) {
            i2c_read_bytes(&bus.sw_handle, _address, recv, recv_len, 0);
        }

        return true;
    }

    if (device_handle == nullptr || bus.bus_handle == nullptr) {
        return false;
    }

    const uint32_t clock_hz = bus.bus_clock != 0 ? bus.bus_clock : 100000U;
    uint32_t timeout_ms = 1U + 16UL * (send_len + recv_len) * 1000UL / clock_hz;
    timeout_ms = MAX(timeout_ms, MAX(_timeout_ms, 5U));

    for (uint8_t attempt = 0; attempt < _retries; attempt++) {
        esp_err_t result;

        if (send_len != 0 && recv_len != 0) {
            result = i2c_master_transmit_receive(
                device_handle,
                send,
                send_len,
                recv,
                recv_len,
                static_cast<int>(timeout_ms));
        } else if (send_len != 0) {
            result = i2c_master_transmit(
                device_handle,
                send,
                send_len,
                static_cast<int>(timeout_ms));
        } else {
            result = i2c_master_receive(
                device_handle,
                recv,
                recv_len,
                static_cast<int>(timeout_ms));
        }

        if (result == ESP_OK) {
            return true;
        }

        if (attempt + 1U < _retries) {
            const esp_err_t reset_result = i2c_master_bus_reset(bus.bus_handle);
            if (reset_result != ESP_OK) {
                return false;
            }
            taskYIELD();
        }
    }

    return false;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::I2CDevice *I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        return nullptr;
    }

    auto *device = NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms);
    if (device == nullptr) {
        return nullptr;
    }

    if (!businfo[bus].soft && device->device_handle == nullptr) {
        delete device;
        return nullptr;
    }

    return device;
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    uint32_t result = 0;

    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1U << i);
        }
    }

    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~get_bus_mask_internal();
}
