#include "mag.h"
#include "stm32f303xc.h"
#include "I2C.h"
#include <math.h>
#include <stdint.h>

#define MAG_ADD            0x1E
#define MAG_WHO_AM_I_REG   0x4F
#define MAG_WHO_AM_I_VAL   0x40
#define MAG_STATUS_REG     0x67
#define MAG_OUT_START      0x68

extern volatile uint32_t system_time_ms;

/*
 * it takes a register address
 * creates a local byte v
 * calls I2C_get_data() to read exactly 1 byte from that register
 * returns that byte
 */
static uint8_t mag_read_reg(uint8_t reg)
{
    uint8_t v = 0;
    I2C_get_data(MAG_ADD, reg, &v, 1);
    return v;
}

static void mag_write_reg(uint8_t reg, uint8_t value)
{
    I2C_write(MAG_ADD, reg, &value, 1); // calls I2C_write() to write exactly 1 byte to that register
}

static int mag_data_ready(void)
{
    uint8_t status = mag_read_reg(MAG_STATUS_REG);
    return ((status & 0x08U) != 0U);   // ZYXDA bit
}

int mag_who_am_i_ok(void)
{
    return (mag_read_reg(MAG_WHO_AM_I_REG) == MAG_WHO_AM_I_VAL);
}

void init_magnetometer(void)
{
    /*
     * CFG_REG_A_M (0x60)
     * bit7 COMP_TEMP_EN = 1
     * bit[3:2] ODR = 01 -> 20 Hz
     * bit[1:0] MD = 00 -> continuous mode
     *
     * 0x84 = 1000 0100
     */
    mag_write_reg(0x60, 0x84);

    /*
     * CFG_REG_B_M (0x61)
     * LPF = 1
     */
    mag_write_reg(0x61, 0x01);

    /*
     * CFG_REG_C_M (0x62)
     * BDU = 1
     */
    mag_write_reg(0x62, 0x10);
}

void set_hard_iron_offsets(magnetometer_data *mag, float ox, float oy, float oz)
{
    mag->offset_x = ox;
    mag->offset_y = oy;
    mag->offset_z = oz;
}

void read_magnetometer(magnetometer_data *mag)
{
    uint8_t buffer[6];

    while (!mag_data_ready()) {
        /* wait for fresh XYZ sample */
    }

    /* this performs the actual I2C read
     * device address = MAG_ADD
     * starting register = MAG_OUT_START | 0x80
     * | 0x80 is commonly used with sensors to enable auto-increment so multiple consecutive registers can be read in one burst
     * destination = buffer
     * number of bytes = 6
     */
    I2C_get_data(MAG_ADD, MAG_OUT_START | 0x80, buffer, 6);

    // combine pairs of bytes into signed 16-bit values
    mag->raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    mag->raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    mag->raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    mag->timestamp = system_time_ms;
}

// this is just math!!
void apply_hard_iron_calibration(magnetometer_data *mag)
{
    mag->mx = (float)mag->raw_x - mag->offset_x;
    mag->my = (float)mag->raw_y - mag->offset_y;
    mag->mz = (float)mag->raw_z - mag->offset_z;
}

// this function smooths the corrected magnetic values
void low_pass_filter(magnetometer_data *mag)
{
    static uint8_t first = 1U;

    if (first) {
        mag->fx = mag->mx;
        mag->fy = mag->my;
        mag->fz = mag->mz;
        first = 0U;
        return;
    }

    mag->fx = ALPHA * mag->mx + (1.0f - ALPHA) * mag->fx;
    mag->fy = ALPHA * mag->my + (1.0f - ALPHA) * mag->fy;
    mag->fz = ALPHA * mag->mz + (1.0f - ALPHA) * mag->fz;
}

// this selects which filtered magnetic components are used to compute heading
void compute_heading(magnetometer_data *mag)
{
    /*
     * Start with this mapping.
     * If headings are mirrored / rotated, swap signs as described below.
     */
    float hx = mag->fx;
    float hy = -mag->fy;
    float heading = atan2f(hy, hx) * (180.0f / 3.14159265f);

    if (heading < 0.0f) {
        heading += 360.0f;
    }

    mag->heading = heading;
}
