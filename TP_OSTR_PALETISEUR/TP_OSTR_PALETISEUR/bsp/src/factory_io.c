/*
 * factory_io.c
 *
 *  Created on: 5 déc. 2020
 *      Author: Laurent
 */

#include "stm32f0xx.h"
#include "factory_io.h"
#include "main.h"
#include "event.h"

/*
 * FACTORY_IO_Actuators_Set
 */
extern uint8_t buffer[7];
void FACTORY_IO_Actuators_Set(uint32_t cmd, uint32_t msk)
{
	uint32_t com = cmd;
	static uint32_t cmd_actual = 0x00;
	cmd = cmd_actual;
	cmd &= ~msk;
	cmd |= com;
	cmd_actual = cmd;

	// Prepare frame buffer
	buffer[0] = TAG_ACTUATORS; 							// Actuators tag

	buffer[1] = (uint8_t) (cmd & 0x000000FF);			// data byte #1
	buffer[2] = (uint8_t)((cmd & 0x0000FF00) >>8U );	// data byte #2
	buffer[3] = (uint8_t)((cmd & 0x00FF0000) >>16U);	// data byte #2
	buffer[4] = (uint8_t)((cmd & 0xFF000000) >>24U);	// data byte #2

	buffer[5] = 0x00;									// CRC (not yet implemented)
	buffer[6] = '\n';									// End byte
}


/*
 * FACTORY_IO_Sensors_Get
 */

uint32_t	FACTORY_IO_Sensors_Get()
{
	uint32_t	sstates;

	// Build 32-bit sensors states representation
	sstates = 0x00000000;

	sstates |= rx_dma_buffer[1];
	sstates |= (rx_dma_buffer[2] <<8U );
	sstates |= (rx_dma_buffer[3] <<16U);
	sstates |= (rx_dma_buffer[4] <<24U);

	return sstates;
}


/*
 * Force Factory IO to send sensors and actuators states
 */

void FACTORY_IO_update(void)
{
	uint8_t	buffer[2];
	uint8_t	n;

	// Prepare frame buffer
	buffer[0] = TAG_UPDATE; 		// Update tag
	buffer[1] = '\n';

	// Send buffer over UART
	for(n=0; n<2; n++)
	{
		while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
		USART2->TDR = buffer[n];
	}
}
