#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "stm32f303xc.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define mag_add 0x1E			//address of magnetometer
#define mag_reg_start_add 0x68	//register where raw data starts
#define mag_data_bytes 6		//number of bytes to read, 2 each for x, y, z raws


typedef struct {
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    float heading;

    uint32_t timestamp;
} magnetometer_data;

void enable_LED_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN; // GPIOE clock
}

void configure_LEDs() {
    // PE8–PE15 as outputs
    for (int i = 8; i <= 15; i++) {
        GPIOE->MODER &= ~(3 << (i * 2));
        GPIOE->MODER |=  (1 << (i * 2)); // output mode
    }
}
void clear_LEDs() {
    for (int i = 8; i <= 15; i++) {
        GPIOE->ODR &= ~(1 << i);
    }
}
void display_heading_led(float heading) {

    clear_LEDs();

    int led_index;

    if (heading >= 337.5 || heading < 22.5)
        led_index = 0; // N
    else if (heading < 67.5)
        led_index = 1; // NE
    else if (heading < 112.5)
        led_index = 2; // E
    else if (heading < 157.5)
        led_index = 3; // SE
    else if (heading < 202.5)
        led_index = 4; // S
    else if (heading < 247.5)
        led_index = 5; // SW
    else if (heading < 292.5)
        led_index = 6; // W
    else
        led_index = 7; // NW

    GPIOE->ODR |= (1 << (8 + led_index));
}
void delay() {
    for (volatile int i = 0; i < 200000; i++);
}


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

	GPIOB->OSPEEDR |= (3 << 12) | (3 << 14); // High speed mode


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

/* read_magnetometer uses I2C to store magnetometer data into a struct
 * *raw_mag -> pointer to struct that will have data put into it
 */
void read_magnetometer(magnetometer_data *raw_mag) {

    uint8_t buffer[mag_data_bytes]; // make space for data

    I2C_get_data(mag_add, mag_reg_start_add | 0x80, buffer, mag_data_bytes); //get and put data into buffer

    // correctly store data into struct
    raw_mag->raw_x = (buffer[0] << 8) | buffer[1];
    raw_mag->raw_z = (buffer[2] << 8) | buffer[3];
    raw_mag->raw_y = (buffer[4] << 8) | buffer[5];
}


void compute_heading(magnetometer_data *raw_mag) {
    raw_mag->heading = atan2((float)raw_mag->raw_y, (float)raw_mag->raw_x) * (180.0f / 3.14159f);

    if (raw_mag->heading < 0)
        raw_mag->heading += 360.0f;
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

void init_magnetometer() {
    uint8_t data;

    // CRA_REG_M (0x00): 15 Hz output rate
    data = 0x14;
    I2C_write(mag_add, 0x00, &data, 1);

    // CRB_REG_M (0x01): gain
    data = 0x20;
    I2C_write(mag_add, 0x01, &data, 1);

    // MR_REG_M (0x02): continuous-conversion mode
    data = 0x00;
    I2C_write(mag_add, 0x02, &data, 1);
}
int main(void)
{
	SCB->CPACR |= (0xF << 20); // enable FPU
    enable_I2C_clocks();
    configure_I2C();
    init_magnetometer();

    enable_LED_clocks();
    configure_LEDs();

    magnetometer_data mag;

    while (1) {
        read_magnetometer(&mag);
        compute_heading(&mag);

        display_heading_led(mag.heading);

        delay();
    }
}
