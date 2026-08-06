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
 *
 * https://www.st.com/resource/en/datasheet/iis2mdc.pdf
 *
 */
#include "AP_Compass_config.h"

#if AP_COMPASS_IIS2MDC_ENABLED

#include "AP_Compass_IIS2MDC.h"

#include <AP_HAL/AP_HAL.h>

// IIS2MDC Registers
#define IIS2MDC_ADDR_CFG_REG_A  0x60
#define IIS2MDC_ADDR_CFG_REG_B  0x61
#define IIS2MDC_ADDR_CFG_REG_C  0x62
#define IIS2MDC_ADDR_STATUS_REG 0x67
#define IIS2MDC_ADDR_OUTX_L_REG 0x68
#define IIS2MDC_ADDR_WHO_AM_I   0x4F

// IIS2MDC Definitions
#define IIS2MDC_WHO_AM_I         0b01000000
#define IIS2MDC_STATUS_REG_READY 0b00001111

// CFG_REG_A
#define COMP_TEMP_EN    (1U << 7)
#define MD_CONTINUOUS   (0U << 0)
#define ODR_100         ((1U << 3) | (1U << 2))

// CFG_REG_B
#define OFF_CANC        (1U << 1)

// CFG_REG_C
#define BDU             (1U << 4)

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IIS2MDC::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = NEW_NOTHROW AP_Compass_IIS2MDC(std::move(dev), force_external, rotation);

    if (sensor == nullptr || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IIS2MDC::AP_Compass_IIS2MDC(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_IIS2MDC::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    if (!check_whoami()) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_A,
                              MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN)) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_B, OFF_CANC)) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_C, BDU)) {
        return false;
    }

    _dev->set_retries(3);
    _dev->set_device_type(DEVTYPE_IIS2MDC);

    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }

    set_rotation(_rotation);

    if (_force_external) {
        set_external(true);
    }

    _dev->register_periodic_callback(
        10000U,
        FUNCTOR_BIND_MEMBER(&AP_Compass_IIS2MDC::timer, void));

    return true;
}

bool AP_Compass_IIS2MDC::check_whoami()
{
    uint8_t whoami = 0;

    if (!_dev->read_registers(IIS2MDC_ADDR_WHO_AM_I, &whoami, 1)) {
        return false;
    }

    return whoami == IIS2MDC_WHO_AM_I;
}

void AP_Compass_IIS2MDC::timer()
{
    struct PACKED {
        uint8_t xout0;
        uint8_t xout1;
        uint8_t yout0;
        uint8_t yout1;
        uint8_t zout0;
        uint8_t zout1;
        uint8_t tout0;
        uint8_t tout1;
    } buffer;

    static uint32_t last_report_ms = 0;

    const float range_scale = 100.0f / 65.535f;

    uint8_t status = 0;
    if (!_dev->read_registers(IIS2MDC_ADDR_STATUS_REG, &status, 1)) {
        return;
    }

    if (!(status & IIS2MDC_STATUS_REG_READY)) {
        return;
    }

    if (!_dev->read_registers(
            IIS2MDC_ADDR_OUTX_L_REG,
            reinterpret_cast<uint8_t *>(&buffer),
            sizeof(buffer))) {
        return;
    }

    const int16_t x = static_cast<int16_t>(
        static_cast<uint16_t>(buffer.xout0) |
        (static_cast<uint16_t>(buffer.xout1) << 8));

    const int16_t y = static_cast<int16_t>(
        static_cast<uint16_t>(buffer.yout0) |
        (static_cast<uint16_t>(buffer.yout1) << 8));

    const int16_t z_raw = static_cast<int16_t>(
        static_cast<uint16_t>(buffer.zout0) |
        (static_cast<uint16_t>(buffer.zout1) << 8));

    const int16_t z = -z_raw;

    Vector3f field{
        x * range_scale,
        y * range_scale,
        z * range_scale
    };

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_report_ms >= 1000U) {
        hal.console->printf(
            "IIS2MDC: raw=(%d,%d,%d) field=(%.1f,%.1f,%.1f)mG "
            "magnitude=%.1fmG status=0x%02X\n",
            static_cast<int>(x),
            static_cast<int>(y),
            static_cast<int>(z),
            static_cast<double>(field.x),
            static_cast<double>(field.y),
            static_cast<double>(field.z),
            static_cast<double>(field.length()),
            static_cast<unsigned>(status));

        last_report_ms = now_ms;
    }
#endif

    accumulate_sample(field);
}

#endif // AP_COMPASS_IIS2MDC_ENABLED
