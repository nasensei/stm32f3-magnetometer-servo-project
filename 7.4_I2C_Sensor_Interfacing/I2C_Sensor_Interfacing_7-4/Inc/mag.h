#ifndef MAG_H
#define MAG_H

#include <stdint.h>

#define ALPHA 0.05f // low-pass filter sens

typedef struct {
	// Raw values
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    // Filtered values
    float fx;
    float fy;
    float fz;

    float heading;
    uint32_t timestamp;
} magnetometer_data;


void init_magnetometer(void);
void read_magnetometer(magnetometer_data *raw_mag);
void compute_heading(magnetometer_data *raw_mag);
void low_pass_filter(magnetometer_data *mag);


#endif
