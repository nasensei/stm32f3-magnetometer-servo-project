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

void SysTick_Handler(void) {
    system_time_ms++;\

}

void init_protocol(void) {

    enable_I2C_clocks();
    configure_I2C();
    init_magnetometer();
    enable_LED_clocks();
    configure_LEDs();
    SysTick_Config(SystemCoreClock / 1000); // 1ms per tick

    serial_init(&serial_console, &SERIAL_HW_USART1_PC4_PC5, 8000000U, 115200U);
    serial_send_string(&serial_console, "UART OK\r\n");
}

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

        char data_2_print[100];
        snprintf(data_2_print, sizeof(data_2_print), "X: %.2f Y: %.2f Z: %.2f Heading: %.2f deg\r\n", mag.fx, mag.fy, mag.fz, mag.heading);

        serial_send_string(&serial_console, data_2_print);
        for (volatile int i = 0; i < 10000; i++);

        // Display on LED ring
        display_heading_led(mag.heading);
        delay();
    }

}




