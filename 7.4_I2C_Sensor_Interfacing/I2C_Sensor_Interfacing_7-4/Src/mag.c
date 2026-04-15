#include "mag.h"
#include "stm32f303xc.h"
#include "I2C.h"
#include <math.h>



// magnetometer in use is LMS303AGR
#define MAG_ADD 0x1E			//address of magnetometer
#define MAG_REG_START 0x68	//register where raw xyz data starts
#define MAG_BYTES 6		//number of bytes to read, 2 each for x, y, z raws


// configure magnetometer to actually work
void init_magnetometer() {
    uint8_t data;

    data = 0x8C; // continuous mode
    I2C_write(0x1E, 0x60, &data, 1);

    data = 0x01; //sensitivity for use on earth
    I2C_write(0x1E, 0x61, &data, 1);

    data = 0x00; // ???
    I2C_write(0x1E, 0x62, &data, 1);
}


/* read_magnetometer uses I2C to store magnetometer data into a struct
 * *raw_mag -> pointer to struct that will have data put into it
 */
void read_magnetometer(magnetometer_data *raw_mag) {

    uint8_t buffer[MAG_BYTES]; // make space for data

    I2C_get_data(MAG_ADD, MAG_REG_START, buffer, MAG_BYTES); //get and put data into buffer

    // correctly store data into struct
    raw_mag->raw_x = (buffer[0] << 8) | buffer[1];
    raw_mag->raw_z = (buffer[2] << 8) | buffer[3];
    raw_mag->raw_y = (buffer[4] << 8) | buffer[5];
}


// Converts raw x/y magnetometer values into 0-360 degree heading
void compute_heading(magnetometer_data *raw_mag) {
    raw_mag->heading = atan2f(raw_mag->fy, raw_mag->fx) * (180.0f / 3.14159f);

    if (raw_mag->heading < 0)
        raw_mag->heading += 360.0f;
}


// Stop header bouncing around, stores in struct under f* values
void low_pass_filter(magnetometer_data *mag)
{
    mag->fx = ALPHA * mag->raw_x + (1.0f - ALPHA) * mag->fx;
    mag->fy = ALPHA * mag->raw_y + (1.0f - ALPHA) * mag->fy;
    mag->fz = ALPHA * mag->raw_z + (1.0f - ALPHA) * mag->fz;
}


