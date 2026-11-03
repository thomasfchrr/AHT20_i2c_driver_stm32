/*
 * aht20.c
 *
 * Reference: Data Sheet AHT20, version V1.0, May 2021.
 */

#include "aht20.h"

static uint8_t tx(AHT20_t *dev, const uint8_t *buf, uint16_t len)
{
	return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, (uint8_t *)buf, len, 100) == HAL_OK ? AHT20_OK : AHT20_ERR_I2C;
}

uint8_t AHT20_ReadStatus(AHT20_t *dev, uint8_t *status)
{
	const uint8_t cmd = AHT20_CMD_STATUS;

	if (tx(dev, &cmd, 1) != AHT20_OK)
	{
		return AHT20_ERR_I2C;
	}
	return HAL_I2C_Master_Receive(dev->hi2c, dev->addr, status, 1, 100) == HAL_OK ? AHT20_OK : AHT20_ERR_I2C;
}

uint8_t AHT20_SoftReset(AHT20_t *dev)
{
	const uint8_t cmd = AHT20_CMD_RESET;

	if (tx(dev, &cmd, 1) != AHT20_OK)
	{
		return AHT20_ERR_I2C;
	}
	HAL_Delay(20);
	return AHT20_OK;
}

uint8_t AHT20_Init(AHT20_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
	uint8_t st;
	const uint8_t init[3] = {AHT20_CMD_INIT, AHT20_INIT_P1, AHT20_INIT_P2};

	dev->hi2c = hi2c;
	dev->addr = addr;

	while (HAL_GetTick() < AHT20_POWER_ON_MS) // 7.1: 100 ms after power-on
	{
	}

	if (AHT20_ReadStatus(dev, &st) != AHT20_OK)
	{
		return AHT20_ERR_I2C;
	}
	if ((st & AHT20_STATUS_MASK) == AHT20_STATUS_MASK)
	{
		return AHT20_OK;
	}

	if (tx(dev, init, 3) != AHT20_OK)
	{
		return AHT20_ERR_I2C;
	}
	HAL_Delay(AHT20_PRE_TRIG_MS);
	if (AHT20_ReadStatus(dev, &st) != AHT20_OK)
	{
		return AHT20_ERR_I2C;
	}
	return (st & AHT20_STATUS_MASK) == AHT20_STATUS_MASK ? AHT20_OK : AHT20_ERR_CAL;
}

uint8_t AHT20_TriggerMeasurement(AHT20_t *dev)
{
	const uint8_t cmd[3] = {AHT20_CMD_TRIGGER, AHT20_TRIGGER_P1, AHT20_TRIGGER_P2};

	return tx(dev, cmd, 3);
}

uint8_t AHT20_WaitReady(AHT20_t *dev, uint32_t timeout_ms)
{
	uint8_t st;
	uint32_t t0 = HAL_GetTick();

	do
	{
		if (AHT20_ReadStatus(dev, &st) != AHT20_OK)
		{
			return AHT20_ERR_I2C;
		}
		if ((st & AHT20_STATUS_BUSY) == 0)
		{
			return AHT20_OK;
		}
	} while ((HAL_GetTick() - t0) <= timeout_ms);

	return AHT20_ERR_TOUT;
}

uint8_t AHT20_CRC8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = AHT20_CRC_INIT;

	while (len--)
	{
		crc ^= *data++;
		for (uint8_t i = 0; i < 8; i++)
		{
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ AHT20_CRC_POLY) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

uint8_t AHT20_ParseFrame(const uint8_t f[7], uint32_t *s_rh, uint32_t *s_t)
{
	if (AHT20_CRC8(f, 6) != f[6])
	{
		return AHT20_ERR_CRC;
	}
	*s_rh = ((uint32_t)f[1] << 12) | ((uint32_t)f[2] << 4) | (f[3] >> 4);
	*s_t = ((uint32_t)(f[3] & 0x0F) << 16) | ((uint32_t)f[4] << 8) | f[5];
	return AHT20_OK;
}

uint8_t AHT20_ReadRaw(AHT20_t *dev, uint32_t *s_rh, uint32_t *s_t)
{
	uint8_t f[7];

	if (HAL_I2C_Master_Receive(dev->hi2c, dev->addr, f, 7, 100) != HAL_OK)
	{
		return AHT20_ERR_I2C;
	}
	return AHT20_ParseFrame(f, s_rh, s_t);
}

float AHT20_ConvertRH(uint32_t s_rh)
{
	return (float)s_rh / 1048576.0f * 100.0f;
}

float AHT20_ConvertTemp(uint32_t s_t)
{
	return (float)s_t / 1048576.0f * 200.0f - 50.0f;
}

int32_t AHT20_ConvertRH_x100(uint32_t s_rh)
{
	return (int32_t)(((uint64_t)s_rh * 10000u) >> 20);
}

int32_t AHT20_ConvertTemp_x100(uint32_t s_t)
{
	return (int32_t)(((uint64_t)s_t * 20000u) >> 20) - 5000;
}

uint8_t AHT20_Measure(AHT20_t *dev, float *temp_c, float *rh)
{
	uint32_t s_rh, s_t;
	uint8_t r;

	r = AHT20_TriggerMeasurement(dev);
	if (r != AHT20_OK)
	{
		return r;
	}
	HAL_Delay(AHT20_MEAS_MS);
	r = AHT20_WaitReady(dev, 100);
	if (r != AHT20_OK)
	{
		return r;
	}
	r = AHT20_ReadRaw(dev, &s_rh, &s_t);
	if (r != AHT20_OK)
	{
		return r;
	}
	*rh = AHT20_ConvertRH(s_rh);
	*temp_c = AHT20_ConvertTemp(s_t);
	return AHT20_OK;
}
