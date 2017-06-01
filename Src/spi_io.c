/*
 *  File: spi_io.c.example
 *  Author: Nelson Lombardo
 *  Year: 2015
 *  e-mail: nelson.lombardo@gmail.com
 *  License at the end of file.
 */

#include "spi_io.h"

/******************************************************************************
 Module Public Functions - Low level SPI control functions
******************************************************************************/

/**
 * MY OWN EXTRA FUNCTIONS START
 */

static SPI_HandleTypeDef* hspi;
static uint32_t timerStartTick;
static uint8_t timerOn = 0;
static uint32_t timerDuration;

void SPI_IO_Attach(SPI_HandleTypeDef* hspi_in){
	hspi = hspi_in;
}

/*
 * MY OWN EXTRA FUNCTIONS END
 */

void SPI_Init (void) {
	if(hspi->State != HAL_SPI_STATE_READY){
		HAL_SPI_Init(hspi);
	}
}

BYTE SPI_RW (BYTE d) {
	BYTE rx;
    HAL_SPI_TransmitReceive(hspi, &d, &rx, 1, 500);
    return rx;
}

void SPI_Release (void) {
    return;
}

inline void SPI_CS_Low (void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 0);
}

inline void SPI_CS_High (void){
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 1);
}

inline void SPI_Freq_High (void) {
    HAL_SPI_DeInit(hspi);
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(hspi);
}

inline void SPI_Freq_Low (void) {
	HAL_SPI_DeInit(hspi);
	hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	HAL_SPI_Init(hspi);
}

void SPI_Timer_On (WORD ms) {
	timerOn = 1;
    timerStartTick = HAL_GetTick();
    timerDuration = ms;
}

inline BOOL SPI_Timer_Status (void) {
//	HAL_
    return ((timerStartTick + timerDuration > HAL_GetTick()) ? TRUE : FALSE);
}

inline void SPI_Timer_Off (void) {
    timerOn = 0;
}

#ifdef SPI_DEBUG_OSC
inline void SPI_Debug_Init(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Port A enable
    PORTA_PCR12 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK  | PORT_PCR_PS_MASK;
    GPIOA_PDDR |= (1 << 12); // Pin is configured as general-purpose output, for the GPIO function.
    GPIOA_PDOR &= ~(1 << 12); // Off
}
inline void SPI_Debug_Mark(void)
{
    GPIOA_PDOR |= (1 << 12); // On
    GPIOA_PDOR &= ~(1 << 12); // Off
}
#endif

/*
The MIT License (MIT)

Copyright (c) 2015 Nelson Lombardo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
