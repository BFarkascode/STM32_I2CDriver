/*
 *  Created on: 03 Nov 2023
 *  Author: BalazsFarkas
 *  Project: STM32_I2CDriver
 *  Processor: STM32L053R8
 *  Program version: 1.0
 *  File: I2CDriver_STM32L0x3.c
 */


/* Custom library to run I2C.
 * Pins are set to be PB8 and PB9.
 * Peripheral runs on APB1 (APB1 is expected to be 8 MHz, otherwise timing is off)
 * Timing/speed is set using TIMINGR values extracted from CubeMX
 * Own address is selected within the I2C config.
 * Analog filter enabled, digital filter disabled
 *
 * CR2 control register is rebuilt during TX
 * Addressing is set to be 7-bits or 10-bits - chosen within the specific TX/RX element. Current version hard-wired for 7-bit addressing.
 * We choose AUTOEND. Hard-wired.
 * TX control is using the START bit in CR2 and using the PE bit in CR1 for reset
 * Address is shifted by 1 (7-bit addressing)
 *
 */

#include "I2CDriver_STM32L0x3.h"
#include "ClockDriver_STM32L0x3.h"
#include "stm32l053xx.h"


//1) Initialization
void I2CConfig(uint8_t dev_own_addr) {

	/*
	 * What happens here?
	 * First, we do the usual by setting the clocking (we enable it, but also choose it to be APB1!), followed by the GPIO setup. We put pull-ups on both the pins.
	 * We disable the I2C and wait a little until the disable takes effect (3 cycles of SYSCLK)
	 * We set the timing by writing to the TIMINGR register.
	 * We put clock stretching and analog filtering on to make the setup more robust.
	 *
	 * Below is the configuration for the I2C for the STM32L0 series
	 * The setting up the I2C is very different compared to M4 devices (what one can easily find on youtube)
	 * No DMA, no CRC and no interrupts are included currently.
	 * Main differences are:
	 * -we need to set the source for the I2C1 in the RCC
	 * -alternative function register values are in the STM32L0 datasheet, not the reference manual
	 * -there is no SWRST bit, we need to use the PE to enable/de-enable the entire peripheral instead
	 * -we don't set standard/fast/fastest mode separate, we use the TIMINGR register
	 * -master/slave switch is automatic: if a START condition is generated, we are in master mode, otherwise slave mode
	 */

	//1)Enable clocking in the RCC, set I2CCLK source clock, enable GPIO clocking - PB8 SCL, PB9 SDA

	RCC->APB1ENR |= (1<<21);					//enable I2C1 clock
	RCC->CCIPR &= ~(1<<12);						//APB1 is selected as I2C source clock
	RCC->CCIPR &= ~(1<<13);						//APB1 is selected as I2C source clock
	RCC->IOPENR |=	(1<<1);						//PORTB clocking

	//2)Set GPIO parameters (mode, speed, pullup) - PB8 SCL, PB9 SDA
	GPIOB->MODER &= ~(1<<16);					//alternate function for PB8
	GPIOB->MODER &= ~(1<<18);					//alternate function for PB9
												//Note: MODER resets to 0xFFFF, so we need to write 0s instead of 1s
	GPIOB->OTYPER |= (1<<8);					//open drain for PB8
	GPIOB->OTYPER |= (1<<9);					//open drain for PB9
	GPIOB->OSPEEDR |= (3<<16);					//high speed PB8
	GPIOB->OSPEEDR |= (3<<18);					//high speed PB9
	GPIOB->PUPDR |= (1<<16);					//pullup PB8
	GPIOB->PUPDR |= (1<<18);					//pullup PB9

	//Note: AFR values are in the device datasheet for L0. For I2C1, they will be AF4 as seen on page 45 of the datasheet.
	GPIOB->AFR[1] |= (4<<0);					//PB8 AF4 setup
	GPIOB->AFR[1] |= (4<<4);					//PB9 AF4 setup

	//3)Set a clock source for the internal clock of the I2C
	I2C1->CR1 &= ~(1<<0);						//disable I2C, used as software reset here - no designated SWRST bit in registers
	Delay_us(1);								//PE should be LOW for at least 3 cycles to take effect! 3 cycles at 8 MHz is 375 ns.
												//config registers are not impacted by the PE reset, only START, STOP, NACK, and various ISR registers

	//4) Set timing - standard mode selected with 8 MHz I2CCLK

	//Standard timing
	I2C1->TIMINGR |= (19<<0);					//SCLL value shall be 0x13 for 8 MHz I2CCLK, Standard mode (see page 727, ref manual)
	I2C1->TIMINGR |= (15<<8);					//SCLH value shall be 0xF for 8 MHz I2CCLK, Standard mode
	I2C1->TIMINGR |= (2<<16);					//SDADEL value shall be 0x2 for 8 MHz I2CCLK, Standard mode
	I2C1->TIMINGR |= (4<<20);					//SCLDEL value shall be 0x4 for 8 MHz I2CCLK, Standard mode
	I2C1->TIMINGR |= (1<<28);					//PRESC value shall be 0x1 for 8 MHz I2CCLK, Standard mode

	//Fast timing
//	I2C1->TIMINGR |= (9<<0);					//SCLL value shall be 0x9 for 8 MHz I2CCLK, Fast mode (see page 727, ref manual)
//	I2C1->TIMINGR |= (3<<8);					//SCLH value shall be 0x3 for 8 MHz I2CCLK, Fast mode
//	I2C1->TIMINGR |= (1<<16);					//SDADEL value shall be 0x1 for 8 MHz I2CCLK, Fast mode
//	I2C1->TIMINGR |= (3<<20);					//SCLDEL value shall be 0x3 for 8 MHz I2CCLK, Fast mode
//	I2C1->TIMINGR &= ~(15<<28);					//PRESC value shall be 0x0 for 8 MHz I2CCLK, Fast mode

	//5) Set own address
	I2C1->OAR1 &= ~(1<<15);						//we disable the own address
	I2C1->OAR1 &= ~(1<<10);						//we choose 7-bit won address
	I2C1->OAR1 |= (dev_own_addr<<1);			//we define the 7-bit own address
	I2C1->OAR1 |= (1<<15);						//we enable the own address

	//6)Clean up the ISR register
	I2C1->ISR |= (1<<0);						//we flush the transmit register

	//7)Enable I2C
	I2C1->CR1 &= ~(1<<12);						//analog filter enabled
	I2C1->CR1 &= ~(1<<17);						//clock stretch enabled
												//this must be kept as such for MASTER mode
	I2C1->CR1 |= (1<<0);						//enable I2C
	Delay_us(1);								//We wait for the setup to take effect
}

//2) Scanning
//This function only sends the address of the slave to the bus, then resets.
//Made specifically for bus scanning.

int I2CSCANNER (uint8_t slave_addr) {
	/*
	 * What happens here?
	 * We scan for our device by sending an address on the bus and check if we have an ACK or a NACK as a response.
	 * Since we only send the address on the bus, we leave the number of bytes to send as constant 0.
	 * We don't increase the addressing here and follow this function up with a serial printf each time we have sent over one address.
	 * We have a big delay to allow all non-blocking functions (such as printing to the serial port) finish before we send the next address down the bus.
	 *
	 */

	uint8_t reply = 0;

	//1)We reset the transmission control register
	I2C1->CR2 = 0x0;								//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

	//2)We set the slave address and the addressing mode
	I2C1->CR2 = (slave_addr << 1);
	I2C1->CR2 &= ~(1<<11);							//7 bits

	//3)We set NBYTES as 0
	I2C1->CR2 |= (0 << 16);							//NBYTES does not include the slave address

	//4)We want an AUTOEND and no RELOAD
	I2C1->CR2 |= (1 << 25);
	I2C1->CR2 &= ~(1 << 24);

	//5)We enable PE and set the START bit
	I2C1->CR2 &= ~(1<<10);							//we write to the slave

	I2C1->CR1 |= (1<<0);							//enable

	I2C1->CR2 |= (1<<13);							//start

	//6)We wait for a reply
	Delay_ms(1);									//This delay is a bit greater than usual so as to cater to printf commands coming afterwards.

	//7)We check the AF flag

	if ((I2C1->ISR & (1<<4)) == (1<<4)) {			//if the NACKF bit is 1, we had a NO ACK!
		reply = 0;									//the init function returns "0" while the NACKF bit is 1
													//NACKF is 1 if there is no acknowledge for the slave address sent over
	} else {
		reply = 1;
	}

	//8)We reset the bus and all the registers in ISR
	I2C1->CR1 &= ~(1<<0);							//we turn off PE and reset the I2C bus
	Delay_us(1);									//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.
	return reply;
}


//3) Transmission
int I2CTX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send) {
	/**
	 *
	 * What happens here?
	 * We put the address into the driver using the (reg<<1) shift
	 * We send a certain number of bytes over the bus to the slave, defined by the number_of_bytes input.
	 * Each byte is to be ACKed by the slave. We ensure this by waiting until the shift registers are emptied.
	 * Data input is an array pointer - pass the function either the array, or the address of the array
	 * Note: there is no interrupt in this TX!
	 *
	 * START bit is automatically generated when we set the appropriate bit. START bit is pulled LOW by the hardware once address is sent.
	 * RD_WRN bit, SADD and NBYTES change is not allowed when START bit is set
	 * All communications are set to be automatic with no focus on specific flag generation. This may need to be changed if we want a good interrupt control.
	 * Writing LOW to the START bit has no effect
	 * "slave_addr" is a 7-bit integer without the write/read bit
	 * START bit is set to LOW after each TX, no matter the ACK/NACK of the address
	 * STOP is generated automatically due to AUTOEND
	 * **/

	//1)We reset the transmission control register
	I2C1->CR2 = 0x0;								//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

	//2)We set the slave address and the addressing mode
	I2C1->CR2 = (slave_addr << 1);					//we write the slave address to the SADD register
													//Note: we are in 7-bit address mode, as it was set above in the I2C config
	I2C1->CR2 &= ~(1<<11);							//7 bits

	//3)We set NBYTES
	I2C1->CR2 |= (number_of_bytes << 16);

	//4)AUTOEND with write as direction, no RELOAD
	I2C1->CR2 |= (1 << 25);
	I2C1->CR2 &= ~(1 << 24);
	I2C1->CR2 &= ~(1<<10);

	//5)We enable PE and set the START bit

	I2C1->CR1 |= (1<<0);							//enable
	I2C1->CR2 |= (1<<13);							//start

	//6)We wait for a reply
	Delay_us(I2C_bus_latency_us);												//waiting. This is to allow the byte and the ACK to cross the bus at 100 kHz

	uint8_t reply = 0;

	if ((I2C1->ISR & (1<<4)) == (1<<4)) {										//if the NACKF bit is 1, it means that we had a NO ACKNOWLEDGE from the slave
		reply = 0;
	} else {																	//if NACK is 0, the slave has been ACKed
		I2C1->CR2 |= (1<< 13);													//we use the START bit to send over the byte
		while (number_of_bytes)
		{
			while(!(I2C1->ISR & (1<<1))) {
				for (volatile uint8_t i = 0x0 ; i < 255 ; i++) {
					//timeout control
					//Note:
					i = i;
				};
				break;

			};										//wait for the TXDR flag to be set
			I2C1->TXDR = (volatile uint8_t) *bytes_to_send++;					//we load the byte and thus will have TXIS LOW until the data byte is cleared
																				//difference between TXIS and TXE is that the TXE is just a state register, not en interrupt
			number_of_bytes--;
			Delay_us(I2C_bus_latency_us);										//25 us wait to transmit the byte and receive an ACK. This is for the 100kHz standard clocking
																				//Note: delay is necessary to avoid clashing of registers: we need to wait until transmission is done before we send a new byte
																				//Note: unfortunately, the NACK bit is active LOW and won't trigger unless there is a NACK. No other flag is available when reload is active.
		}
		reply = 1;
	}

//8)We reset the bus and all the registers in ISR
	I2C1->CR1 &= ~(1<<0);														//we turn off PE and reset the I2C bus
	Delay_us(1);																//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.
	return reply;
}

//4) Reception
int I2CRX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t* bytes_received) {
	/*
	 *
	 * Expects a certain number of bytes over the bus from the slave
	 * Each byte is automatically ACKed by the master, except the last one - NACK will go HIGH!
	 * Data input is an array pointer - pass the function either the array, or the address of the array
	 * STOP is generated automatically due to AUTOEND
	 *
	 */

		//1)We reset the transmission control register
		I2C1->CR2 = 0x0;														//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

		//2)We set the slave address and the addressing mode
		I2C1->CR2 = (slave_addr << 1);											//we write the slave address to the SADD register
																				//Note: we are in 7-bit address mode, as it was set above in the I2C config
		I2C1->CR2 &= ~(1<<11);													//7 bits

		//3)We set NBYTES
		I2C1->CR2 |= (number_of_bytes << 16);

		//4)AUTOEND with write as direction, no RELOAD
		I2C1->CR2 |= (1 << 25);
		I2C1->CR2 &= ~(1 << 24);

		//5)We read now!
		I2C1->CR2 |= (1<<10);

		//6)We enable PE and set the START bit

		I2C1->CR1 |= (1<<0);													//enable
		I2C1->CR2 |= (1<<13);													//start

		//7)We wait for a reply
		Delay_us(I2C_bus_latency_us);											//latency wait

		uint8_t reply = 0;

		if ((I2C1->ISR & (1<<4)) == (1<<4)) {									//if the NACKF bit is 1, it means that we had a NO ACKNOWLEDGE from the slave
			reply = 0;
		} else {																//if NACK is 0, the slave has been ACKed
			while (number_of_bytes) {
				while (!(I2C1->ISR & (1<<2)));									//we wait for the RXNE flag to go HIGH, indicating that we have something in the RXDN register
				*bytes_received = I2C1->RXDR;									//we dereference the pointer, thus we can give it a value
				bytes_received++;												//this is technically an address. A pointer is technically a memory address. Here we step through the array at the address.
				number_of_bytes--;
			}
			reply = 1;
		}

		//8)We reset the bus and all the registers in ISR
		I2C1->CR1 &= ~(1<<0);													//we turn off PE and reset the I2C bus
		Delay_us(1);
		return reply;
}

//5) Readout
//The following function takes an array of registers to read out and then replaces the registers with their readout values

void I2CReadout(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer) {
	/*
	 * What happens here?
	 * We write and then read out, using the same array.
	 * Be aware that a Tx needs to be used to tell the slave that we want to read and from which register.
	 * Then, we put out driver into Rx mode, send over the address and await how the sensor is automatically starting to send over the data.
	 *
	 * */

	for (int i = 0; i < number_of_readouts; i++) {
		I2CTX(slave_addr, 1, &data_buffer[i]);
		Delay_us(1);
		I2CRX(slave_addr, 1, &data_buffer[i]);
	}
}
