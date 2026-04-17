#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "I2C.h"
#include "mag.h"
#include "leds.h"
#include "stm32f303xc.h"
#include "serial.h"



#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// magnetometer in use is LMS303AGR

uint32_t SystemCoreClock = 8000000; //jank workaround for jank directory stuff
volatile uint32_t system_time_ms = 0;

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

    serial_init(&serial_console, &SERIAL_HW_USART1_PC4_PC5, 8000000U, 115200U);
    serial_send_string(&serial_console, "UART OK\r\n");
}

int main(void)
{
    SCB->CPACR |= (0xF << 20);
    init_protocol();

    magnetometer_data mag = {0};
    int16_t min_x = 32767, min_y = 32767, min_z = 32767;
    int16_t max_x = -32768, max_y = -32768, max_z = -32768;

    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;

    if (!mag_who_am_i_ok()) {
        serial_send_string(&serial_console, "MAG WHO_AM_I FAIL\r\n");
        while (1) { }
    }

    /* replace these with your measured offsets */
    set_hard_iron_offsets(&mag, 0.0f, 0.0f, 0.0f);

    while (1) {
        read_magnetometer(&mag);
        if (mag.raw_x < min_x) min_x = mag.raw_x;
        if (mag.raw_y < min_y) min_y = mag.raw_y;
        if (mag.raw_z < min_z) min_z = mag.raw_z;

        if (mag.raw_x > max_x) max_x = mag.raw_x;
        if (mag.raw_y > max_y) max_y = mag.raw_y;
        if (mag.raw_z > max_z) max_z = mag.raw_z;

        offset_x = (max_x + min_x) * 0.5f;
        offset_y = (max_y + min_y) * 0.5f;
        offset_z = (max_z + min_z) * 0.5f;

        set_hard_iron_offsets(&mag, offset_x, offset_y, offset_z);

        apply_hard_iron_calibration(&mag);
        low_pass_filter(&mag);
        compute_heading(&mag);

        char data_2_print[120];
        snprintf(data_2_print, sizeof(data_2_print),
                 "raw=(%d,%d,%d) corr=(%.1f,%.1f,%.1f) head=%.1f\r\n",
                 mag.raw_x, mag.raw_y, mag.raw_z,
                 mag.fx, mag.fy, mag.fz,
                 mag.heading);

        serial_send_string(&serial_console, data_2_print);

        display_heading_led(mag.heading);
        delay();
    }
}
