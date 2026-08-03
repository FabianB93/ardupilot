/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

DeviceBus::DeviceBus(uint8_t _thread_priority) :
    semaphore(),
    thread_priority(_thread_priority)
{
#ifdef BUSDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif
}

/*
  Per-bus callback thread.

  I2C DeviceBus tasks are pinned to CPU 1 when they are created. The
  ArduPilot main loop remains on CPU 0. The task sleeps using a FreeRTOS
  task notification instead of repeatedly calling delay_microseconds().
 */
void DeviceBus::bus_thread(void *arg)
{
#ifdef BUSDEBUG
    printf("%s:%d\\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    auto *binfo = static_cast<DeviceBus *>(arg);

    while (true) {
        uint64_t now = AP_HAL::micros64();

        /*
         * Run callbacks that are due. Missed periods are skipped instead
         * of replayed so a delayed bus task cannot build up a callback
         * backlog and monopolise the CPU.
         */
        for (callback_info *callback = binfo->callbacks;
             callback != nullptr;
             callback = callback->next) {

            if (now < callback->next_usec) {
                continue;
            }

            do {
                callback->next_usec += callback->period_usec;
            } while (now >= callback->next_usec);

            if (binfo->semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                callback->cb();
                binfo->semaphore.give();
            }

            /*
             * Give other ready tasks of equal priority an opportunity to
             * run after a potentially lengthy bus callback.
             */
            taskYIELD();

            now = AP_HAL::micros64();
        }

        /*
         * Determine the earliest next callback deadline.
         */
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback_info *callback = binfo->callbacks;
             callback != nullptr;
             callback = callback->next) {

            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
            }
        }

        /*
         * No callback registered yet: sleep for up to 50 ms. A newly
         * registered callback wakes the task using xTaskNotifyGive().
         */
        if (next_needed == 0) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
            continue;
        }

        now = AP_HAL::micros64();

        if (next_needed <= now) {
            /*
             * Work is already due. Yield once and immediately recalculate
             * instead of sleeping for a complete RTOS tick.
             */
            taskYIELD();
            continue;
        }

        uint64_t remaining_us = next_needed - now;

        /*
         * Hybrid wait:
         *
         * 1. For longer waits, block most of the interval using a FreeRTOS
         *    task notification. This releases CPU 1 and still allows a new
         *    callback registration to wake the task immediately.
         *
         * 2. Leave approximately one millisecond for a high-resolution
         *    microsecond delay. This avoids rounding a 2500 us deadline up
         *    to three RTOS ticks.
         */
        if (remaining_us > 1500U) {
            const uint32_t coarse_ms =
                static_cast<uint32_t>((remaining_us - 1000U) / 1000U);

            if (coarse_ms > 0U) {
                (void)ulTaskNotifyTake(
                    pdTRUE,
                    pdMS_TO_TICKS(coarse_ms));

                /*
                 * A notification or normal timeout may have changed the
                 * earliest deadline. Recalculate in the outer loop.
                 */
                continue;
            }
        }

        /*
         * The remaining interval is short. delay_microseconds() preserves
         * sub-millisecond timing precision. It is intentionally limited to
         * at most about 1.5 ms, so the DeviceBus task cannot busy-wait for
         * long periods.
         */
        now = AP_HAL::micros64();
        if (next_needed > now) {
            remaining_us = next_needed - now;
            hal.scheduler->delay_microseconds(
                static_cast<uint32_t>(remaining_us));
        }
    }
}

AP_HAL::Device::PeriodicHandle
DeviceBus::register_periodic_callback(uint32_t period_usec,
                                      AP_HAL::Device::PeriodicCb cb,
                                      AP_HAL::Device *_hal_device)
{
#ifdef BUSDEBUG
    printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    if (period_usec == 0) {
        return nullptr;
    }

    auto *callback = NEW_NOTHROW callback_info;
    if (callback == nullptr) {
        return nullptr;
    }

    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    callback->next = nullptr;

    if (!thread_started) {
        hal_device = _hal_device;

        char name[configMAX_TASK_NAME_LEN] {};

        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name,
                     sizeof(name),
                     "APM_I2C:%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name,
                     sizeof(name),
                     "APM_SPI:%u",
                     hal_device->bus_num());
            break;

        default:
            snprintf(name,
                     sizeof(name),
                     "APM_BUS:%u",
                     hal_device->bus_num());
            break;
        }

#ifdef BUSDEBUG
        printf("%s:%d Thread Start\n", __PRETTY_FUNCTION__, __LINE__);
#endif

        BaseType_t task_result;

        if (hal_device->bus_type() ==
            AP_HAL::Device::BUS_TYPE_I2C) {
            /*
             * Keep the high-rate I2C sensor task away from APM_MAIN on
             * CPU 0. Previous watchdog traces showed CPU 1 idle while
             * APM_I2C:0 occupied CPU 0.
             */
            task_result =
                xTaskCreatePinnedToCore(
                    DeviceBus::bus_thread,
                    name,
                    Scheduler::DEVICE_SS,
                    this,
                    thread_priority,
                    &bus_thread_handle,
                    1);
        } else {
            task_result =
                xTaskCreate(
                    DeviceBus::bus_thread,
                    name,
                    Scheduler::DEVICE_SS,
                    this,
                    thread_priority,
                    &bus_thread_handle);
        }

        if (task_result != pdPASS) {
            bus_thread_handle = nullptr;
            delete callback;
            return nullptr;
        }

        thread_started = true;
    }

    /*
     * Add the callback after successful task creation.
     */
    callback->next = callbacks;
    callbacks = callback;

    /*
     * Wake the DeviceBus task immediately so it recalculates its next
     * deadline instead of waiting for the current timeout.
     */
    if (bus_thread_handle != nullptr) {
        xTaskNotifyGive(bus_thread_handle);
    }

    return callback;
}

/*
 * Adjust the timer for the next call. This must be called from the bus
 * thread, otherwise it races with the callback scheduler.
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h,
                             uint32_t period_usec)
{
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle ||
        h == nullptr ||
        period_usec == 0) {
        return false;
    }

    auto *callback = static_cast<callback_info *>(h);
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    return true;
}
