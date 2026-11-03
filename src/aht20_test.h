/*
 * aht20_test.h
 *
 * Test suite of the AHT20 driver, checked against the AHT20 datasheet V1.0.
 * Results are printed with printf.
 */

#ifndef AHT20_AHT20_TEST_H_
#define AHT20_AHT20_TEST_H_

#include "aht20.h"

// Each test returns the number of failed checks (0 = pass)
uint32_t AHT20_Test_Identity(AHT20_t *dev);   // address 0x38 ACK, wrong address NACK
uint32_t AHT20_Test_Status(AHT20_t *dev);     // status byte (table 9), calibration bit
uint32_t AHT20_Test_CRC(AHT20_t *dev);        // CRC-8 known answer + real frame
uint32_t AHT20_Test_Conversion(AHT20_t *dev); // formulas of section 8, frame decoding
uint32_t AHT20_Test_Measure(AHT20_t *dev);    // trigger, busy bit, 80 ms timing
uint32_t AHT20_Test_Reset(AHT20_t *dev);      // soft reset keeps the calibration
uint32_t AHT20_Test_Ranges(AHT20_t *dev);     // values in range, repeatability

// Runs everything, prints a summary, returns total number of failed checks
uint32_t AHT20_Test_RunAll(AHT20_t *dev);

#endif /* AHT20_AHT20_TEST_H_ */
