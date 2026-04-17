#ifndef MAG_H
#define MAG_H

#include <stdint.h>

#define ALPHA 0.05f

typedef struct {
    // Raw mag values
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    // Hard-iron corrected mag values
    float mx;
    float my;
    float mz;

    // Filtered mag values
    float fx;
    float fy;
    float fz;

    // Hard-iron offsets
    float offset_x;
    float offset_y;
    float offset_z;

    // Accelerometer raw
    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;

    // Accelerometer in g-ish units / filtered
    float ax;
    float ay;
    float az;

    float roll;
    float pitch;
    float heading;

    uint32_t timestamp;
} magnetometer_data;

void init_magnetometer(void);
void read_magnetometer(magnetometer_data *raw_mag);
void low_pass_filter(magnetometer_data *mag);
void apply_hard_iron_calibration(magnetometer_data *mag);
void set_hard_iron_offsets(magnetometer_data *mag, float ox, float oy, float oz);
void compute_heading(magnetometer_data *mag);

/* accelerometer / tilt compensation */
void init_accelerometer(void);
void read_accelerometer(magnetometer_data *mag);
void compute_tilt_angles(magnetometer_data *mag);
void compute_tilt_compensated_heading(magnetometer_data *mag);

int mag_who_am_i_ok(void);

#endif
