#include "stm32f303xc.h"
#include "I2C.h"


// enable the clocks for GPIOB (SCL and SDA), and I2C
void enable_I2C_clocks() {

	RCC->AHBENR |=RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;

}
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

	GPIOB->OSPEEDR |= (3 << 12) | (3 << 14); // High speed mode for TIMINGR (see below)


	// I2C CONFIG
	// turn off first
	I2C1->CR1 &= ~I2C_CR1_PE;

	// set clock speed
    I2C1->TIMINGR = 0x00310309; // 400 kHz. standard according to Khit
	// re-enable I2C
	I2C1->CR1 |= I2C_CR1_PE;

}

/* I2C_get_data receives data using I2C
 * address -> address of device to get data from
 * reg2read -> which register of aforementioned device to get data
 * *data_buff -> pointer to variable just to temp store data
 * bytes2read -> how many bytes to read
 */

void I2C_get_data(uint8_t address, uint8_t reg2read, uint8_t *data_buff, uint8_t bytes2read) {

	// WRITE INTO I2C REGISTERS
    I2C1->CR2 = (address << 1) | (1 << 16); // writes device address and NBYTES=1
    I2C1->CR2 |= I2C_CR2_START; // start I2C

    while (!(I2C1->ISR & I2C_ISR_TXIS)); // check interrupt status (TXIS) to see if ready to read yet
    I2C1->TXDR = reg2read; //copy data to transmit

    while (!(I2C1->ISR & I2C_ISR_TC)); // wait until transfer is complete (ISR_TC)

    // READ FROM I2C REGSITERS
    I2C1->CR2 = (address << 1) | (bytes2read << 16) | I2C_CR2_RD_WRN; // RD_WRN set to 1 - master requests a read transfer
    I2C1->CR2 |= I2C_CR2_START; // start again but for read

    for (int i = 0; i < bytes2read; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE)); // wait until data is received (check RXNE)
        data_buff[i] = I2C1->RXDR; // copy data from receive register to designated variable
    }

    I2C1->CR2 |= I2C_CR2_STOP; // stop I2C
}

void I2C_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t nbytes) {

    // Configure transfer: address + number of bytes (reg + data)
    I2C1->CR2 = (address << 1) | ((nbytes + 1) << 16); // +1 for register
    I2C1->CR2 |= I2C_CR2_START;

    // Send register address first
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;

    // Send data bytes
    for (int i = 0; i < nbytes; i++) {
        while (!(I2C1->ISR & I2C_ISR_TXIS));
        I2C1->TXDR = data[i];
    }

    // Wait until transfer complete
    while (!(I2C1->ISR & I2C_ISR_TC));

    // Generate STOP
    I2C1->CR2 |= I2C_CR2_STOP;
}
