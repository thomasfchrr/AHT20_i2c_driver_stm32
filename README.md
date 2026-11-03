# AHT20 I2C driver for STM32 (HAL)

C driver for the ASAIR (Aosong) **AHT20** humidity and temperature sensor over **I2C**, built on the STM32 HAL, with a complete on-target test suite checked against the datasheet ([docs/DS_AHT20.pdf](docs/DS_AHT20.pdf), V1.0, May 2021).

Developed and tested on a **NUCLEO-F411RE** (STM32F411RE, I2C1 at 100 kHz, USART2 at 115200 for logs).

## Files

| File | Role |
|------|------|
| `src/aht20.h` / `src/aht20.c` | The driver |
| `src/aht20_test.h` / `src/aht20_test.c` | Test suite (printf report) |
| `docs/DS_AHT20.pdf` | ASAIR datasheet (not covered by the MIT license) |

## Features

- Status read (command `0x71`, table 9: busy bit 7, calibration bit 3)
- Initialization: 100 ms power-on wait, calibration check, init command if needed
- Measurement: trigger (`0xAC 0x33 0x00`), busy polling, 7 byte frame read
- **CRC-8** check of every frame (init `0xFF`, polynomial `x^8 + x^5 + x^4 + 1`)
- Conversions of datasheet section 8:
  - `RH [%] = S_RH / 2^20 * 100`
  - `T [degC] = S_T / 2^20 * 200 - 50`
  - float versions and integer-only versions (0.01 %RH, 0.01 degC, no FPU needed)
- Frame decoder (`AHT20_ParseFrame`) usable without hardware
- Helper `AHT20_Measure` (trigger, 80 ms wait, busy poll, read, CRC, conversion)
- Soft reset (`0xBA`)

Not implemented: dew point, heater or any feature not in the datasheet.

## Usage

Copy the four files from `src/` into your project and add that folder to the include path. `aht20.h` includes `stm32f4xx_hal.h`, change it for another STM32 family.

```c
#include "aht20.h"

AHT20_t aht;

if (AHT20_Init(&aht, &hi2c1, AHT20_I2C_ADDR) != 0) { /* 1: I2C error, 3: not calibrated */ }

float t, rh;
if (AHT20_Measure(&aht, &t, &rh) == 0)
{
    /* t in degC, rh in %RH */
}
```

The address is the 8 bit HAL form of the 7 bit address `0x38`. Return codes: `0` ok, `1` I2C error, `2` timeout, `3` not calibrated, `4` CRC error.

Step by step, if you need your own timing:

```c
AHT20_TriggerMeasurement(&aht);
HAL_Delay(80);                        /* datasheet 7.4: wait more than 80 ms */
AHT20_WaitReady(&aht, 100);           /* status bit 7 = 0 */
uint32_t s_rh, s_t;
AHT20_ReadRaw(&aht, &s_rh, &s_t);     /* 20 bit raw values, CRC checked */
int32_t t_x100  = AHT20_ConvertTemp_x100(s_t);   /* 2085 = 20.85 degC */
int32_t rh_x100 = AHT20_ConvertRH_x100(s_rh);    /* 3644 = 36.44 %RH */
```

### Notes from the datasheet and from the hardware

- **Calibration bit.** Section 7.4 says the status & `0x18` must be `0x18`, but table 9 only defines bit 3 as the calibration flag and bit 4 as reserved. A real AHT20 answers `0x08` (and `0x0C` after a measurement) once calibrated, so the driver only tests bit 3 (`AHT20_STATUS_CAL`). Testing `0x18` makes `Init` fail on a perfectly good sensor.
- **Init command.** The datasheet only says to "initialize the 0x1B, 0x1C, 0x1E registers" when the sensor is not calibrated. The driver sends `0xBE 0x08 0x00`, the sequence of the ASAIR reference code. It is not exercised on the tested sensor, which was already calibrated.
- **Soft reset `0xBA`** is also not in this datasheet (ASAIR reference code).
- **Timing.** The datasheet asks for more than 80 ms between trigger and read. The sensor used here was ready after about 43 ms, the driver keeps the 80 ms of the datasheet.
- **Self heating.** Section 4.4: keep a measurement period above 1 s and the I2C clock between 10 kHz and 400 kHz.
- The sensor needs a 10 uF decoupling capacitor and pull-ups on SDA and SCL (sections 5.1 and 5.3). The sensor shares I2C1 with other devices without problem: its address `0x38` is fixed.

## Test suite

```c
#include "aht20_test.h"

AHT20_Test_RunAll(&aht);   /* prints a PASS / FAIL line per test, returns the number of failed checks */
```

`printf` must be redirected (for example `__io_putchar` to a UART).

| Test | What is checked | Datasheet |
|------|-----------------|-----------|
| Identity | ACK at 0x38, no ACK at 0x39, error code on a wrong address | 7.3 |
| Status | calibration bit set, busy bit clear when idle, `Init` on a calibrated device | table 9, 7.4 |
| CRC | known answer `crc(BE EF) = 0x92`, every single bit flip of a frame is detected (56 cases), CRC sent by the sensor equals the computed one | 7.4 step 4 |
| Conversion | endpoints and midpoints of both formulas, float vs integer over the 20 bit range, 3 frames decoded bit by bit | 7.4, 8.1, 8.2 |
| Measure | busy bit set right after the trigger, ready within 90 ms, idle and calibrated afterwards, `AHT20_Measure` | 7.4 steps 2-3 |
| Reset | calibration kept after the soft reset, measurement still works | (reference code) |
| Ranges | T and RH inside the operating range, repeatability over 5 shots spaced by 1 s | tables 1, 2, 4.4 |

### Result on hardware (NUCLEO-F411RE, room conditions)

```
[PASS] Identity
    status=0x0C
[PASS] Status
[PASS] CRC
[PASS] Conversion
    conversion time: 43 ms
    T=2085 cC  RH=3644 c%
[PASS] Measure
[PASS] Reset
    5 shots: dT=165 mC dRH=263 mRH
[PASS] Ranges
===== ALL PASSED (0 failed checks) =====
```

### Limits of the tests

- The conversion tests check the formulas against hand computed values (for example `S = 2^19` gives 50.00 %RH and 50.00 degC). No accuracy test against a reference instrument was done: the datasheet gives +-0.3 degC and +-2 %RH, which cannot be verified without one.
- The power-on calibration path (`Init` sending the init command) cannot be forced and is not tested.
- The I2C speed was only tested at 100 kHz.

## License

The code is released under the [MIT License](LICENSE). The datasheet is copyright ASAIR and is included for reference only.
