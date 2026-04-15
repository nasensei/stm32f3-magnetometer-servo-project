#include "stm32f303xc.h"
#include "leds.h"

void enable_LED_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
}

void configure_LEDs() {
    // configure PE8–PE15 to output mode
    for (int i = 8; i <= 15; i++) {
        GPIOE->MODER &= ~(3 << (i * 2));
        GPIOE->MODER |=  (1 << (i * 2));
    }
}
void clear_LEDs() {
	// just in case
    for (int i = 8; i <= 15; i++) {
        GPIOE->ODR &= ~(1 << i);
    }
}

// Display current heading on the LED ring for visualisation
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

// For LED ring use
void delay() {
    for (volatile int i = 0; i < 100000; i++); // 100000 MAGICAL NUMBER???
}
