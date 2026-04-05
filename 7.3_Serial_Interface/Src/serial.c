#include "serial.h"
#include "stm32f303xc.h"

#define USART1_TX_PIN   4U   /* PC4 -> USART1_TX */
#define USART1_RX_PIN   5U   /* PC5 -> USART1_RX */
#define USART_AF_NUM    7U   /* AF7 for USART1 */


// configuring alternate function
static void gpio_set_alternate_function(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af) {
	uint32_t reg_index = pin / 8U; // AFR[0] for pins 0..7, AFR[1] for 8..15
	uint32_t shift = (pin % 8U) * 4U; // find bit position of this pin's 4-bit AF field
	gpio->AFR[reg_index] &= ~(0xFU << shift); // clear the old AF its for this pin
	gpio->AFR[reg_index] |=  (af << shift); // write new AF value into that field
}

static uint8_t serial_checksum(uint8_t msg_type, const uint8_t *payload, uint8_t payload_size)
{
    /* Simple 8-bit XOR checksum */
    uint8_t checksum = 0U;
    checksum ^= payload_size;
    checksum ^= msg_type;

    for (uint8_t i = 0U; i < payload_size; i++) {
        checksum ^= payload[i];
    }

    return checksum;
}


void serial_init(uint32_t peripheral_clock_hz, uint32_t baud) {
	// enable clock
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    // enable USART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PC4, PC5 -> alternate function mode
    GPIOC->MODER &= ~((0x3U << (USART1_TX_PIN * 2U)) |
                      (0x3U << (USART1_RX_PIN * 2U)));

    GPIOC->MODER |=  ((0x2U << (USART1_TX_PIN * 2U)) |
                      (0x2U << (USART1_RX_PIN * 2U)));

    /* Optional: high speed */
    GPIOC->OSPEEDR |= ((0x3U << (USART1_TX_PIN * 2U)) |
                     	(0x3U << (USART1_RX_PIN * 2U)));

    /* No pull-up / pull-down */
    GPIOC->PUPDR &= ~((0x3U << (USART1_TX_PIN * 2U)) |
                      (0x3U << (USART1_RX_PIN * 2U)));

    /* Select AF7 on PC4/PC5 */
    gpio_set_alternate_function(GPIOC, USART1_TX_PIN, USART_AF_NUM);
    gpio_set_alternate_function(GPIOC, USART1_RX_PIN, USART_AF_NUM);

    /* Reset USART config: default 8 data bits, no parity, 1 stop bit */
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    /*
     * BRR for oversampling by 16.
     * This project skeleton usually starts on the default clock unless you later add clock setup.
    */
    USART1->BRR = (peripheral_clock_hz + (baud / 2U)) / baud;

    /* Enable transmitter, receiver, UART */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

}

void serial_write_byte(uint8_t byte)
{
    while ((USART1->ISR & USART_ISR_TXE) == 0U) {
        /* wait */
    }

    USART1->TDR = byte;
}
