/*
 *  Created on: 03 Nov 2023
 *  Author: BalazsFarkas
 *  Project: STM32_I2CDriver
 *  Processor: STM32L053R8
 *  Compiler: ARM-GCC (STM32 IDE)
 *  HEader version: 1.0
 *  File: I2CDriver_STM32L0x3.h
 */

#ifndef INC_I2CDRIVER_CUSTOM_H_
#define INC_I2CDRIVER_CUSTOM_H_

#include "stdint.h"
#include "ClockDriver_STM32L0x3.h"

//LOCAL CONSTANT
//bus latency
static const uint8_t I2C_bus_latency_us = 0x5A;			//it takes 90 um (0x5A) for a byte and an ACK to cross the bus at 100 kHz
														//it takes 25 um (0x19) for a byte and an ACK to cross the bus at 400 kHz

//FUNCTION PROTOTYPES
void I2CConfig(uint8_t dev_own_addr);
int I2CSCANNER (uint8_t slave_addr);
int I2CTX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send);
int I2CRX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t* bytes_received);
void I2CReadout(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer);


#endif /* INC_I2CDRIVER_CUSTOM_H_ */
