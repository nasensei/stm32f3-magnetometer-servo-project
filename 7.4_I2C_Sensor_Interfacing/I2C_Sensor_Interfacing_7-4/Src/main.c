#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "I2C.h"
#include "mag.h"
#include "leds.h"
#include "stm32f303xc.h"
#include "serial.h"
#include "system_stm32f3xx.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// magnetometer in use is LMS303AGR

uint32_t SystemCoreClock = 8000000; //jank workaround for jank directory stuff
volatile uint32_t system_time_ms = 0;



int main(void)
{
	SCB->CPACR |= (0xF << 20); // enable FPU
	init_protocol(); //enable/configure everything else

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

void SysTick_Handler(void) {
    system_time_ms++;
}

void init_protocol(void) {

    enable_I2C_clocks();
    configure_I2C();
    init_magnetometer();
    enable_LED_clocks();
    configure_LEDs();
    SysTick_Config(SystemCoreClock / 1000); // 1ms per tick

}

