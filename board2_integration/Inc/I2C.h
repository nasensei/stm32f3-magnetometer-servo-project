#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void enable_I2C_clocks(void);
void configure_I2C(void);
void I2C_get_data(uint8_t address, uint8_t reg2read, uint8_t *data_buff, uint8_t bytes2read);
void I2C_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t nbytes);

#endif
