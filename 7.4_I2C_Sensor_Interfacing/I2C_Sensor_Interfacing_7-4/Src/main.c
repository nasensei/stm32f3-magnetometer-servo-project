#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "I2C.h"
#include "mag.h"
#include "leds.h"
#include "stm32f303xc.h"



#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// magnetometer in use is LMS303AGR



int main(void)
{
	// Various init/config functions
	SCB->CPACR |= (0xF << 20); // enable FPU

    enable_I2C_clocks();
    configure_I2C();
    init_magnetometer();
    enable_LED_clocks();
    configure_LEDs();

    // Create struct to pass around
    magnetometer_data mag = {0};


    // Compass heading update loop
    while (1) {
    	// Get data from IMU and store into struct
        read_magnetometer(&mag);

        // Filter it, store in struct
        low_pass_filter(&mag);

        // Compute heading, store in struct
        compute_heading(&mag);

        // Display on LED ring
        display_heading_led(mag.heading);
        delay();
    }
}
