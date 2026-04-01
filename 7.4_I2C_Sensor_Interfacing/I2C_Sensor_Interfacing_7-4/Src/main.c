

#include <stdint.h>
#include "stm32f303xc.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


// enable the clocks for GPIOB (SCL and SDA), and I2C 
void enable_I2C_clocks() {

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_APB1ENR_I2C1EN


// Configure PB6 (SCL) and PB7 (SDA) for I2C
void configure_I2C() {

	// GPIOB PINS CONFIG
	// set PB6 & 7 to alternate function mode
	GPIOB->MODER &= ~((3 << 12) | (3 << 14));
	GPIOB->MODER |=  ((2 << 12) | (2 << 14)); 

	// configure OTYPER for open drain
	GPIOB->OTYPER |= (1 << 6) | (1 << 7);

	// force pull up resistors so SCL and SDA are high by default for I2C
	GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
	GPIOB->PUPDR |=  ((1 << 12) | (1 << 14));

	// make AF mode set earlier use I2C functionality
	GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28));
	GPIOB->AFR[0] |=  ((4 << 24) | (4 << 28));
	
	
	// I2C CONFIG
	// turn off first
	I2C1->CR1 &= ~I2C_CR1_PE;
	
	// set clock speed
	I2C1->TIMINGR = 0x2000090E; // not sure about value, corresponds to 100kHz according to chatgpt
	
	// re-enable I2C
	I2C1->CR1 |= I2C_CR1_PE;

}


int main(void)
{
	enable_I2C_clocks();
	configure_I2C();

	}
}
