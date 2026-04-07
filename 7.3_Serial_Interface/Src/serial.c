#include "serial.h"
#include "stm32f303xc.h"

#define USART1_TX_PIN   4U   /* PC4 -> USART1_TX */
#define USART1_RX_PIN   5U   /* PC5 -> USART1_RX */
#define USART_AF_NUM    7U   /* AF7 for USART1 */
// maybe we can change these timeout values in the future
#define SERIAL_TX_TIMEOUT 1000000U
#define SERIAL_RX_TIMEOUT 1000000U

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

    // switch to high speed, we can adjust this later
    GPIOC->OSPEEDR |= ((0x3U << (USART1_TX_PIN * 2U)) |
                     	(0x3U << (USART1_RX_PIN * 2U)));

    // No pull-up / pull-down
    GPIOC->PUPDR &= ~((0x3U << (USART1_TX_PIN * 2U)) |
                      (0x3U << (USART1_RX_PIN * 2U)));

    // Select AF7 on PC4/PC5
    gpio_set_alternate_function(GPIOC, USART1_TX_PIN, USART_AF_NUM);
    gpio_set_alternate_function(GPIOC, USART1_RX_PIN, USART_AF_NUM);

    // Reset USART config: start bit + 8 bit word length + stop bit
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    /*
     * BRR for oversampling by 16.
     * we consider this for default clock
    */
    USART1->BRR = (peripheral_clock_hz + (baud / 2U)) / baud;

    // Enable transmitter, receiver, UART
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

}

bool serial_write_byte(uint8_t byte){
	uint32_t timeout= SERIAL_TX_TIMEOUT;

	// check if USART is enable for transmit
	if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE)) {
	   return false;
	}

	// wait until transmit data register is empty
    while ((USART1->ISR & USART_ISR_TXE) == 0U) {
        /// WAIT for USART1 to be ready to accept new byte
    	// the below if block will help stopping the loop from waiting forever
    	if (timeout-- == 0U) {
    		return false;
    	}
    }
    // put the byte into the transmit data register
    USART1->TDR = byte;
    return true;
}

bool serial_read_byte(uint8_t *out_byte) {
	uint32_t timeout = SERIAL_RX_TIMEOUT;

	if (out_byte == NULL) {
		return false;
	}
	// check if USART is enabled for receive
	if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_RE)) != (USART_CR1_UE | USART_CR1_RE)) {
	   return false;
	}
	// RXNE = Receive data register not empty
	while ((USART1->ISR & USART_ISR_RXNE) == 0) {
		// before start reading, make sure that a byte has actually arrived
		// stopping the loop from waiting forever
		if (timeout-- == 0U) {
			return false;
		}
	}
	// USART1->RDR = receive data register
	// 0xFF = 11111111, mask that keeps only the 8 lowest bits
	*out_byte = (uint8_t)(USART1->RDR & 0xFFU);
	return true;
}

bool serial_send_bytes(const uint8_t *data, size_t length) {
	uint32_t timeout = SERIAL_TX_TIMEOUT;

	if (data == NULL && length > 0) {
		return false;
	}
	for (size_t i = 0U; i < length; i++) {
		if (!serial_write_byte(data[i])) {
			return false;
		}
	}
	// wait until the final byte has fully left, TC = transmission complete flag
	while ((USART1->ISR & USART_ISR_TC) == 0U) {
		//it's not ready yet, wait until the flag transmission complete is UP
		//stopping the loop from waiting forever
		if (timeout-- == 0U) {
			return false;
		}
	}
	return true;
}

bool serial_recv_bytes(uint8_t *data, size_t length) {
	if (data == NULL && length > 0U) {
		return false;
	}
	for (size_t i = 0U; i < length; i++) {
		if (!serial_read_byte(&data[i])) {
			return false;
		}
	}
	return true;
}

bool serial_send_string(const char *str) {
	uint32_t timeout = SERIAL_TX_TIMEOUT;

	if (str == NULL) {
		return false;
	}
	while (*str != '\0') {
		if (!serial_write_byte((uint8_t)(*str))) {
			return false;
		}
		str++;
	}
	while ((USART1->ISR & USART_ISR_TC) == 0U) {
	    // wait until the TC flag is UP
		// stop the loop from waiting forever
		if (timeout-- == 0U) {
			return false;
		}
    }
	return true;
}

bool serial_send_msg(uint8_t msg_type, const void *payload, uint8_t payload_size){
	if ((payload_size > SERIAL_MAX_PAYLOAD) ||
	        ((payload == NULL) && (payload_size > 0U))) {
	        return false;
	}
	const uint8_t *payload_bytes = (const uint8_t *)payload;
	uint8_t packet[SERIAL_MAX_PAYLOAD + 5U]; // the buffer large enough for the worst case
	uint8_t checksum = serial_checksum(msg_type, payload_bytes, payload_size);

	// PACKET FORMAT:
	// [START][SIZE][TYPE][PAYLOAD...][CHECKSUM][STOP]
	packet[0] = SERIAL_START_BYTE;
	packet[1] = payload_size;
	packet[2] = msg_type;
	//writing byte
	for (uint8_t i = 0U; i < payload_size; i++) {
	    packet[3U + i] = payload_bytes[i];
	}
	packet[3U + payload_size] = checksum;
	packet[4U + payload_size] = SERIAL_STOP_BYTE;

	// send the bytes
	if (!serial_send_bytes(packet, (size_t)payload_size + 5U)) {
	    return false;
	}
	return true;
}
