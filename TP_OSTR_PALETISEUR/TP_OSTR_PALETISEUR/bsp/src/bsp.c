/*
 * bsp.c
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#include "bsp.h"
#include "factory_io.h"

/*
 * BSP_LED_Init()
 * Initialize LED pin (PA5) as  High-Speed Push-Pull Output
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_On()
 * Turn ON LED on PA5
 */

void BSP_LED_On()
{
	GPIOA->BSRR = GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}

/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t	BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOC->IDR & GPIO_IDR_13) == GPIO_IDR_13)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}

	return state;
}

/*
 * BSP_Console_Init()
 * USART2 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PA2 (AF1)
 * RX -> PA3 (AF1)
 */

extern uint8_t buffer[7];
void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);

	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Actual BaudRate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Actual BaudRate = 115246.0984 -> 0.04% error (better choice)

	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 139;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Setup RX on DMA Channel 5

	// Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Reset DMA1 Channel 5 configuration
	DMA1_Channel5->CCR = 0x00000000;

	// Set direction Peripheral -> Memory
	DMA1_Channel5->CCR &= ~DMA_CCR_DIR;

	// Peripheral is USART2 RDR
	DMA1_Channel5->CPAR = (uint32_t)&USART2->RDR;

	// Peripheral data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_PSIZE_Pos);

	// Disable auto-increment Peripheral address
	DMA1_Channel5->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel5->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

	// Enable auto-increment Memory address
	DMA1_Channel5->CCR |= DMA_CCR_MINC;

	// Set Memory Buffer size
	DMA1_Channel5->CNDTR = FRAME_LENGTH;

	// DMA mode is circular
	DMA1_Channel5->CCR |= DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART2->CR3 |= USART_CR3_DMAR;

	// Reset DMA1 Channel 4 configuration
	DMA1_Channel4->CCR = 0x00000000;

	// Set direction Memory -> peripherique
	DMA1_Channel4->CCR |= DMA_CCR_DIR;

	// Peripheral is USART2 TDR
	DMA1_Channel4->CPAR = (uint32_t)&USART2->TDR;

	// Peripheral data size is 8-bit (byte)
	DMA1_Channel4->CCR |= (0x00 <<DMA_CCR_PSIZE_Pos);

	// Disable auto-increment Peripheral address
	DMA1_Channel4->CCR &= ~DMA_CCR_PINC;

	// Memory is tx_dma_buffer
	DMA1_Channel4->CMAR = (uint32_t)buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel4->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

	// Enable auto-increment Memory address
	DMA1_Channel4->CCR |= DMA_CCR_MINC;

	// Set Memory Buffer size
	//DMA1_Channel5->CNDTR = 8;

	// DMA mode is circular
	DMA1_Channel4->CCR &= ~DMA_CCR_CIRC;

	DMA1_Channel4->CCR |= DMA_CCR_TCIE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}

void I2C1_Init()
{

	// Select SYSCLK as I2C1 clock (48MHz)
	RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

	// Enable I2C1 clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	// Make sure I2C1 is disabled
	I2C1->CR1 &= ~I2C_CR1_PE;

	// Reset I2C1 Configuration to default values
	I2C1->CR1 	  = 0x00000000;
	I2C1->CR2 	  = 0x00000000;
	I2C1->TIMINGR = 0x00000000;

	// Configure timing for 100kHz, 50% duty cycle
	I2C1->TIMINGR |= ((4 -1) <<I2C_TIMINGR_PRESC_Pos); // Clock prescaler /4 -> 12MHz
	I2C1->TIMINGR |= (60 	 <<I2C_TIMINGR_SCLH_Pos);  // High half-period = 5µs
	I2C1->TIMINGR |= (60     <<I2C_TIMINGR_SCLL_Pos);  // Low  half-period = 5µs

	// Enable I2C1
	I2C1->CR1 |= I2C_CR1_PE;
}
uint8_t	BSP_I2C1_Read( uint8_t device_address,
                       uint8_t register_address,
                       uint8_t *buffer,
                       uint8_t nbytes )
{
	uint32_t 	timeout;	// Flag waiting timeout
	uint8_t		n;			// Loop counter

	// Set device address
	I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
	I2C1->CR2 |= ((device_address <<1U) <<I2C_CR2_SADD_Pos);

        // Set I2C in Write mode
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;

	// Transfer NBYTES = 1, no AUTOEND
	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= (1 <<16U);
	I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	// Start I2C transaction
	I2C1->CR2 |= I2C_CR2_START;

	// Wait for TXIS with timeout
	timeout = 100000;
	while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	{
		timeout--;
		if (timeout == 0) return 1;
	}

	// Send Register address
	I2C1->TXDR = register_address;

	// Wait for TC with timeout
	timeout = 100000;
	while (((I2C1->ISR) & I2C_ISR_TC) != I2C_ISR_TC)
	{
		timeout--;
		if (timeout == 0) return 2;
	}

	// Set I2C in Read mode
	I2C1->CR2 |= I2C_CR2_RD_WRN;

	// Transfer NBYTES, no AUTOEND
	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= (nbytes <<16U);
	I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	// Re-Start transaction
	I2C1->CR2 |= I2C_CR2_START;

	n = nbytes;

	while (n>0)
	{
		// Wait for RXNE with timeout
		timeout = 100000;
		while (((I2C1->ISR) & I2C_ISR_RXNE) != I2C_ISR_RXNE)
		{
			timeout--;
			if (timeout == 0) return 3;
		}

		// Store data into buffer
		*buffer = I2C1->RXDR;
		buffer++;
		n--;
	}

	// Generate STOP condition
	I2C1->CR2 |= I2C_CR2_STOP;

	// Wait for STOPF with timeout
	timeout = 100000;
	while (((I2C1->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	{
		timeout--;
		if (timeout == 0) return 4;
	}

	// Return success
	return 0;
}
uint8_t	BSP_I2C1_Write( uint8_t device_address,
                        uint8_t register_address,
                        uint8_t *buffer, uint8_t nbytes )
{
	uint32_t 	timeout;	// Flag waiting timeout
	uint8_t		n;		// Loop counter

	// Set device address
	I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
	I2C1->CR2 |= ((device_address <<1U) <<I2C_CR2_SADD_Pos);

	// Set I2C in Write mode
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;

	// Transfer NBYTES, with AUTOEND
	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= ((nbytes+1) <<16U);
	I2C1->CR2 |= I2C_CR2_AUTOEND;

	// Clear STOPF flag
	I2C1->ICR |= I2C_ICR_STOPCF;

	// Start I2C transaction
	I2C1->CR2 |= I2C_CR2_START;

	// Wait for TXIS with timeout
	timeout = 100000;
	while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	{
		timeout--;
		if (timeout == 0) return 1;
	}

	// Send register address
	I2C1->TXDR = register_address;

	n = nbytes;

	while(n>0)
	{
		// Wait for TXIS with timeout
		timeout = 100000;
		while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
		{
			timeout--;
			if (timeout == 0) return 2;
		}

		// Send data
		I2C1->TXDR = *buffer;
		buffer++;
		n--;
	}

	// Wait for STOPF with timeout
	timeout = 100000;
	while (((I2C1->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	{
		timeout--;
		if (timeout == 0) return 3;
	}

	// Return success
	return 0;
}
