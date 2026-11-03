/*
 * aht20_test.c
 *
 * Datasheet references: Data Sheet AHT20 V1.0 (May 2021)
 */

#include "aht20_test.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static uint32_t g_fail; // failed checks of the running test

#define CHECK(cond, ...)                                                                                               \
	do                                                                                                                 \
	{                                                                                                                  \
		if (!(cond))                                                                                                   \
		{                                                                                                              \
			g_fail++;                                                                                                  \
			printf("    FAIL L%d: ", __LINE__);                                                                        \
			printf(__VA_ARGS__);                                                                                       \
			printf("\r\n");                                                                                            \
		}                                                                                                              \
	} while (0)

static void report(const char *name, uint32_t fails)
{
	printf("[%s] %s\r\n", fails ? "FAIL" : "PASS", name);
}

/* ------------------------------------------------------------------ */

uint32_t AHT20_Test_Identity(AHT20_t *dev)
{
	g_fail = 0;
	// 7.3: the 7 bit address is 0x38
	CHECK(HAL_I2C_IsDeviceReady(dev->hi2c, AHT20_I2C_ADDR, 3, 20) == HAL_OK, "no ACK at 0x38");
	CHECK(HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(0x39 << 1), 1, 5) != HAL_OK, "unexpected ACK at 0x39");

	AHT20_t bad = *dev;
	uint8_t st;
	bad.addr = (uint8_t)(0x50 << 1);
	CHECK(AHT20_ReadStatus(&bad, &st) == AHT20_ERR_I2C, "wrong address must return error 1");
	CHECK(AHT20_Init(&bad, dev->hi2c, bad.addr) == AHT20_ERR_I2C, "Init on wrong address must return 1");
	HAL_I2C_DeInit(dev->hi2c);
	HAL_I2C_Init(dev->hi2c); // clear the bus error state
	report("Identity", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_Status(AHT20_t *dev)
{
	uint8_t st = 0;

	g_fail = 0;
	CHECK(AHT20_ReadStatus(dev, &st) == AHT20_OK, "status read");
	// table 9: bit 3 = 1 means calibrated (7.4 step 1 quotes 0x18 but bit 4 is reserved)
	CHECK((st & AHT20_STATUS_CAL) == AHT20_STATUS_CAL, "status=0x%02X, calibration bit not set", st);
	CHECK((st & AHT20_STATUS_BUSY) == 0, "device busy while idle, status=0x%02X", st);
	CHECK(AHT20_Init(dev, dev->hi2c, dev->addr) == AHT20_OK, "Init on a calibrated device");
	printf("    status=0x%02X\r\n", st);
	report("Status", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_CRC(AHT20_t *dev)
{
	uint8_t f[7];
	uint32_t rh, t;

	(void)dev;
	g_fail = 0;

	// CRC-8, init 0xFF, polynomial 0x31 (7.4 step 4): known answer crc(0xBE 0xEF) = 0x92
	const uint8_t v[2] = {0xBE, 0xEF};
	CHECK(AHT20_CRC8(v, 2) == 0x92, "crc(BE EF)=0x%02X expected 0x92", AHT20_CRC8(v, 2));
	CHECK(AHT20_CRC8(v, 0) == 0xFF, "crc of nothing is the init value 0xFF");

	// a frame with a valid CRC decodes, one flipped bit anywhere is rejected
	f[0] = 0x1C;
	f[1] = 0x6A;
	f[2] = 0xB5;
	f[3] = 0x3C;
	f[4] = 0x4D;
	f[5] = 0xE2;
	f[6] = AHT20_CRC8(f, 6);
	CHECK(AHT20_ParseFrame(f, &rh, &t) == AHT20_OK, "valid frame");
	for (uint8_t i = 0; i < 7; i++)
	{
		for (uint8_t b = 0; b < 8; b++)
		{
			f[i] ^= (uint8_t)(1 << b);
			CHECK(AHT20_ParseFrame(f, &rh, &t) == AHT20_ERR_CRC, "bit %u of byte %u not detected", b, i);
			f[i] ^= (uint8_t)(1 << b);
		}
	}

	// live frame: the 7th byte sent by the sensor must be the CRC of the first 6
	uint8_t live[7];
	AHT20_t *d = dev;
	CHECK(AHT20_TriggerMeasurement(d) == AHT20_OK, "trigger");
	HAL_Delay(AHT20_MEAS_MS);
	CHECK(AHT20_WaitReady(d, 100) == AHT20_OK, "wait");
	CHECK(HAL_I2C_Master_Receive(d->hi2c, d->addr, live, 7, 100) == HAL_OK, "read 7 bytes");
	CHECK(AHT20_CRC8(live, 6) == live[6], "sensor crc=0x%02X computed=0x%02X", live[6], AHT20_CRC8(live, 6));
	report("CRC", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_Conversion(AHT20_t *dev)
{
	uint8_t f[7];
	uint32_t rh, t;

	(void)dev;
	g_fail = 0;

	// 8.1: RH = S_RH / 2^20 * 100, 8.2: T = S_T / 2^20 * 200 - 50
	CHECK(AHT20_ConvertRH_x100(0) == 0, "RH(0)");
	CHECK(AHT20_ConvertRH_x100(0x80000) == 5000, "RH(2^19) must be 50.00");
	CHECK(AHT20_ConvertRH_x100(0xFFFFF) == 9999, "RH(2^20-1)=%ld", (long)AHT20_ConvertRH_x100(0xFFFFF));
	CHECK(AHT20_ConvertTemp_x100(0) == -5000, "T(0) must be -50.00");
	CHECK(AHT20_ConvertTemp_x100(0x80000) == 5000, "T(2^19) must be 50.00");
	CHECK(AHT20_ConvertTemp_x100(0x40000) == 0, "T(2^18) must be 0.00");
	CHECK(AHT20_ConvertTemp_x100(0xFFFFF) == 14999, "T(2^20-1)=%ld", (long)AHT20_ConvertTemp_x100(0xFFFFF));
	CHECK(fabsf(AHT20_ConvertRH(0x80000) - 50.0f) < 0.001f, "float RH(2^19)");
	CHECK(fabsf(AHT20_ConvertTemp(0x80000) - 50.0f) < 0.001f, "float T(2^19)");
	CHECK(fabsf(AHT20_ConvertTemp(0x40000)) < 0.001f, "float T(2^18)");

	// float and integer versions agree over the whole 20 bit range
	for (uint32_t s = 0; s <= 0xFFFFF; s += 0x1111)
	{
		CHECK(fabsf(AHT20_ConvertRH(s) - AHT20_ConvertRH_x100(s) / 100.0f) < 0.02f, "RH mismatch at 0x%05lX",
		      (unsigned long)s);
		CHECK(fabsf(AHT20_ConvertTemp(s) - AHT20_ConvertTemp_x100(s) / 100.0f) < 0.02f, "T mismatch at 0x%05lX",
		      (unsigned long)s);
	}

	// frame layout (7.4): status, RH[19:12], RH[11:4], RH[3:0]|T[19:16], T[15:8], T[7:0], crc
	f[0] = 0x1C;
	f[1] = 0x80;
	f[2] = 0x00;
	f[3] = 0x08;
	f[4] = 0x00;
	f[5] = 0x00;
	f[6] = AHT20_CRC8(f, 6);
	CHECK(AHT20_ParseFrame(f, &rh, &t) == AHT20_OK && rh == 0x80000 && t == 0x80000, "frame 1 rh=0x%05lX t=0x%05lX",
	      (unsigned long)rh, (unsigned long)t);
	f[1] = 0x6A;
	f[2] = 0xB5;
	f[3] = 0x3C;
	f[4] = 0x4D;
	f[5] = 0xE2;
	f[6] = AHT20_CRC8(f, 6);
	CHECK(AHT20_ParseFrame(f, &rh, &t) == AHT20_OK && rh == 0x6AB53 && t == 0xC4DE2, "frame 2 rh=0x%05lX t=0x%05lX",
	      (unsigned long)rh, (unsigned long)t);
	f[1] = 0xFF;
	f[2] = 0xFF;
	f[3] = 0xFF;
	f[4] = 0xFF;
	f[5] = 0xFF;
	f[6] = AHT20_CRC8(f, 6);
	CHECK(AHT20_ParseFrame(f, &rh, &t) == AHT20_OK && rh == 0xFFFFF && t == 0xFFFFF, "frame 3 full scale");
	report("Conversion", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_Measure(AHT20_t *dev)
{
	uint8_t st = 0;
	uint32_t t0, dt;
	float t, rh;

	g_fail = 0;
	CHECK(AHT20_TriggerMeasurement(dev) == AHT20_OK, "trigger");
	t0 = HAL_GetTick();
	// 7.4 step 3: bit 7 = 1 while the measurement runs
	CHECK(AHT20_ReadStatus(dev, &st) == AHT20_OK && (st & AHT20_STATUS_BUSY), "busy bit not set, status=0x%02X", st);
	CHECK(AHT20_WaitReady(dev, 200) == AHT20_OK, "never ready");
	dt = HAL_GetTick() - t0;
	// the host must wait more than 80 ms, the sensor is done within that time
	CHECK(dt <= AHT20_MEAS_MS + 10, "measurement took %lu ms, expected <= 90", (unsigned long)dt);
	printf("    conversion time: %lu ms\r\n", (unsigned long)dt);
	uint32_t a, b;
	CHECK(AHT20_ReadRaw(dev, &a, &b) == AHT20_OK, "read frame");

	// after the read the device is idle and still calibrated
	CHECK(AHT20_ReadStatus(dev, &st) == AHT20_OK && (st & AHT20_STATUS_BUSY) == 0 && (st & AHT20_STATUS_CAL),
	      "idle+calibrated expected, status=0x%02X", st);

	// high level helper
	CHECK(AHT20_Measure(dev, &t, &rh) == AHT20_OK, "AHT20_Measure");
	printf("    T=%ld cC  RH=%ld c%%\r\n", (long)(t * 100), (long)(rh * 100));
	report("Measure", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_Reset(AHT20_t *dev)
{
	uint8_t st = 0;
	float t, rh;

	g_fail = 0;
	// command 0xBA is not in the datasheet V1.0, it comes from the ASAIR reference code
	CHECK(AHT20_SoftReset(dev) == AHT20_OK, "soft reset");
	CHECK(AHT20_ReadStatus(dev, &st) == AHT20_OK, "status after reset");
	CHECK((st & AHT20_STATUS_MASK) == AHT20_STATUS_MASK, "calibration lost after reset, status=0x%02X", st);
	CHECK(AHT20_Measure(dev, &t, &rh) == AHT20_OK, "measure after reset");
	report("Reset", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_Ranges(AHT20_t *dev)
{
	float t, rh, tmin = 1000, tmax = -1000, hmin = 1000, hmax = -1000;

	g_fail = 0;
	// 4.4: data cycle > 1 s to avoid self heating
	for (int i = 0; i < 5; i++)
	{
		CHECK(AHT20_Measure(dev, &t, &rh) == AHT20_OK, "measure %d", i);
		// operating range: -40..85 degC, 0..100 %RH (tables 1, 2)
		CHECK(t > -40.0f && t < 85.0f, "T out of range %ld cC", (long)(t * 100));
		CHECK(rh >= 0.0f && rh <= 100.0f, "RH out of range %ld c%%", (long)(rh * 100));
		if (t < tmin)
			tmin = t;
		if (t > tmax)
			tmax = t;
		if (rh < hmin)
			hmin = rh;
		if (rh > hmax)
			hmax = rh;
		HAL_Delay(1000);
	}
	printf("    5 shots: dT=%ld mC dRH=%ld mRH\r\n", (long)((tmax - tmin) * 1000), (long)((hmax - hmin) * 1000));
	CHECK(tmax - tmin < 1.0f, "T spread %ld mC", (long)((tmax - tmin) * 1000));
	CHECK(hmax - hmin < 5.0f, "RH spread %ld mRH", (long)((hmax - hmin) * 1000));
	report("Ranges", g_fail);
	return g_fail;
}

uint32_t AHT20_Test_RunAll(AHT20_t *dev)
{
	uint32_t total = 0;

	printf("\r\n===== AHT20 driver test =====\r\n");
	total += AHT20_Test_Identity(dev);
	total += AHT20_Test_Status(dev);
	total += AHT20_Test_CRC(dev);
	total += AHT20_Test_Conversion(dev);
	total += AHT20_Test_Measure(dev);
	total += AHT20_Test_Reset(dev);
	total += AHT20_Test_Ranges(dev);
	printf("===== %s (%lu failed checks) =====\r\n", total ? "FAILED" : "ALL PASSED", (unsigned long)total);
	return total;
}
