/*
 * mcp3208.c
 *
 *  Created on: Aug 18, 2012
 *      Author: AmosSam
 *     Version: 0.1
 *
 *  Edited on Aug 2, 2014
 *       Editor: ChrisL8
 *       Version 0.2
 *       Increased all "delay(10)" statments to "delay(15)"
 *       Because 10 is too short for waitcnt.
 *       COMMENTED OUT all delay entries, because in CMM on Propeller they are not even needed.
 */

#include <propeller.h>

/**
 * @brief sets pin state, int pin, to high
 */
void pinHigh(int pin)
{
    OUTA |= (1 << pin);
    DIRA |= (1 << pin);
}

/**
 * @brief sets pin state, int pin, to low
 */
void pinLow(int pin)
{
    OUTA &= ~(1 << pin);
    DIRA |= (1 << pin);
}

/**
 * @brief sets pin direction, int pin, to input
 */
void pinInput(int pin)
{
    DIRA &= ~(1 << pin);
}

/**
 * @brief sets pin direction, int pin, to output
 */
void pinOutput(int pin)
{
    DIRA |= (1 << pin);
}

/**
 * @brief reads pin state, int pin
 */
int pinRead(int pin)
{
    DIRA &= ~(1 << pin);
    return (INA & (1 << pin)) ? 1 : 0;
}

/**
 * @brief sets pin state, int pin, to int state
 */
int pinWrite(int pin, int state)
{
    if (state)
        OUTA |= (1 << pin);
    else
        OUTA &= ~(1 << pin);
    DIRA |= (1 << pin);
    return (OUTA & (1 << pin)) != 0;
}

/**
 * @brief delay function, more precise on Propeller (?)
 */
void delay(int us)
{
    waitcnt(us*(CLKFREQ/1000000)+CNT);
}

/**
 * @brief sets pin, int pin, state to high, delays specified time, int d,
 * and then puts pin low, and delays specified time, int d1
 */
void pinPulseHL(int pin, int d, int d1)
{
   pinHigh(pin);
   if (d > 10)
	   delay(d);
   pinLow(pin);
   if (d1 > 10)
	   delay(d1);
}

/**
 * @brief sets pin, int pin, state to low, delays specified time, int d,
 * and then puts pin high, and delays specified time, int d1
 */
void pinPulseLH(int pin, int d, int d1)
{
   pinHigh(pin);
   if (d > 10)
	   delay(d);
   pinLow(pin);
   if (d1 > 10)
	   delay(d1);
}

/**
 * @brief Reads value from mcp3208/mcp3204 ADC
 * @details This function reads data from mcp3208/mcp3204 ADC
 * over SPI, with DIN and DOUT pins tied together to single
 * pin on Propeller with 3.3K resistor in between pins on ADC
 *
 * @param channel What channel to read? 0 - 7
 * @param dinout Pin number on which DIN and DOUT are connected
 * @param clk Pin number on which CLK is connected
 * @param cs Pin number on which CS is connected
 * @returns Value read from mcp3208/mcp3204 ADC
 */
int readADC(int channel, int dinout, int clk, int cs)
{
	int i;
	int AdcResult;
	int setup;

	//Setting up pins

	// In case pin was already been low, we put it high
	// so we can initiate communication after setting up pins
	pinOutput(cs);
	pinHigh(cs);

	pinOutput(dinout);
	pinLow(dinout);

	pinOutput(clk);
	pinLow(clk);

	pinLow(cs);  	// Active chip select by setting pin low
	//delay(15);

	// Sending configuration to device
	setup = channel | 0b11000;
	for(i=0; i < 5;i++) {
		pinPulseHL(clk, 10, 0);
		if ((setup & 0b10000) == 0b10000)
			pinHigh(dinout);
		else
			pinLow(dinout); // is MSB != 0
		setup <<= 1;  // shift left
		//delay(15);
	}

	pinPulseHL(clk, 10, 10); //Empty clock, for sampling

	pinPulseHL(clk, 10, 10); //Device returns low, NULL bit, we ignore it...

	pinInput(dinout);

	// read ADC result 12 bit
	AdcResult=0;
	for(i=0;i<12;i++) {
		// We are sending pulse, clock signal, to ADC, because on falling edge it will return data...
		pinPulseHL(clk, 10, 0);
		// Shifting bit to left, to make room for current one...
		AdcResult<<=1;
		AdcResult=AdcResult | (pinRead(dinout) & 0x01);
		//delay(15);
	}
	pinHigh(cs);
	return(AdcResult);
}

/**
 * @brief Reads specified number of values from mcp3208/mcp3204 ADC
 * @details This function reads specified number of times
 * data from mcp3208/mcp3204 ADC over SPI,
 * with DIN and DOUT pins tied together to single
 * pin on Propeller with 3.3K resistor in between pins on ADC
 *
 * @param channel What channel to read? 0 - 7
 * @param dinout Pin number on which DIN and DOUT are connected
 * @param clk Pin number on which CLK is connected
 * @param cs Pin number on which CS is connected
 * @param samples Number of samples that will be gathered, and then taken average before it's returned
 * @returns Value read from mcp3208/mcp3204 ADC
 */
int readADCAverage(int channel, int dinout, int clk, int cs, int samples)
{
	int i, k = 0;

	for (i=0;i<samples;i++)
		k = k + readADC(channel, dinout, clk, cs);
	return k/samples;
}

