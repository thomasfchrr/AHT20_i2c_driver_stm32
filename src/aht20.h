/*
 * aht20.h
 *
 * Driver for the ASAIR AHT20 humidity and temperature sensor (I2C), STM32 HAL.
 * Reference: Data Sheet AHT20, version V1.0, May 2021.
 */

#ifndef AHT20_AHT20_H_
#define AHT20_AHT20_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

// 7 bit address 0x38 (datasheet 7.3), shifted for the HAL
#define AHT20_I2C_ADDR (0x38 << 1)

// Commands (datasheet 7.4)
#define AHT20_CMD_STATUS  0x71 // read the status byte
#define AHT20_CMD_TRIGGER 0xAC // trigger measurement, parameters 0x33 0x00
#define AHT20_TRIGGER_P1  0x33
#define AHT20_TRIGGER_P2  0x00
// Not in this datasheet, taken from the ASAIR application note / reference code
#define AHT20_CMD_INIT  0xBE // initialize calibration, parameters 0x08 0x00
#define AHT20_INIT_P1   0x08
#define AHT20_INIT_P2   0x00
#define AHT20_CMD_RESET 0xBA // soft reset

// Status byte (table 9)
#define AHT20_STATUS_BUSY 0x80 // bit 7: 1 = measuring, 0 = idle
#define AHT20_STATUS_CAL  0x08 // bit 3: 1 = calibrated
// Datasheet 7.4 step 1 says "status & 0x18 must equal 0x18" but table 9 only defines bit 3 as the calibration
// flag (bit 4 is reserved). A real AHT20 answers 0x08 once calibrated, so only bit 3 is tested.
#define AHT20_STATUS_MASK AHT20_STATUS_CAL

// Timings in ms (datasheet 7.1 and 7.4)
#define AHT20_POWER_ON_MS 100
#define AHT20_PRE_TRIG_MS 10
#define AHT20_MEAS_MS     80

// Return codes of the functions below
#define AHT20_OK       0
#define AHT20_ERR_I2C  1
#define AHT20_ERR_TOUT 2
#define AHT20_ERR_CAL  3
#define AHT20_ERR_CRC  4

#define AHT20_CRC_INIT 0xFF
#define AHT20_CRC_POLY 0x31 // x^8 + x^5 + x^4 + 1

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;
} AHT20_t;

// Waits for power-on time, checks the calibration bit (table 9), runs the init command if needed.
// Returns 0, 1 I2C error, 3 not calibrated
uint8_t AHT20_Init(AHT20_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

uint8_t AHT20_ReadStatus(AHT20_t *dev, uint8_t *status);
uint8_t AHT20_SoftReset(AHT20_t *dev);
uint8_t AHT20_TriggerMeasurement(AHT20_t *dev);
// Polls the status until busy (bit 7) is cleared. Returns 0, 1 I2C error, 2 timeout
uint8_t AHT20_WaitReady(AHT20_t *dev, uint32_t timeout_ms);

// CRC-8 (init 0xFF, polynomial 0x31) of len bytes
uint8_t AHT20_CRC8(const uint8_t *data, uint8_t len);
// Decodes a 7 byte frame (status, 20 bit humidity, 20 bit temperature, crc). Returns 0 or 4 on CRC error
uint8_t AHT20_ParseFrame(const uint8_t frame[7], uint32_t *s_rh, uint32_t *s_t);
// Reads the 7 byte frame and decodes it. Returns 0, 1 I2C, 4 CRC
uint8_t AHT20_ReadRaw(AHT20_t *dev, uint32_t *s_rh, uint32_t *s_t);

// Conversions of datasheet section 8
float AHT20_ConvertRH(uint32_t s_rh);         // RH [%]   = S_RH / 2^20 * 100
float AHT20_ConvertTemp(uint32_t s_t);        // T  [degC] = S_T / 2^20 * 200 - 50
int32_t AHT20_ConvertRH_x100(uint32_t s_rh);  // 0.01 %RH, integer only
int32_t AHT20_ConvertTemp_x100(uint32_t s_t); // 0.01 degC, integer only

// Full cycle: trigger, wait 80 ms, wait ready, read, check CRC, convert
uint8_t AHT20_Measure(AHT20_t *dev, float *temp_c, float *rh);

#endif /* AHT20_AHT20_H_ */
