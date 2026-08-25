#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize SCCB by creating a dedicated I2C master bus.
 * For ArduPilot, SCCB_Use_Port() is preferred when the camera shares an
 * already initialized ArduPilot I2C bus.
 */
int SCCB_Init(int pin_sda, int pin_scl);

/*
 * Reuse an I2C master bus owned and initialized by ArduPilot.
 */
int SCCB_Use_Port(int i2c_num);

/*
 * Remove SCCB device handles. An ArduPilot-owned bus is never deleted.
 */
int SCCB_Deinit(void);

/*
 * Probe one SCCB/I2C slave address.
 * Returns ESP_OK (0) when the device acknowledges, otherwise an esp_err_t.
 */
int SCCB_Probe(uint8_t slv_addr);

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif
