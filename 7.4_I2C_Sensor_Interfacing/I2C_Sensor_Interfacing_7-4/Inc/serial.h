#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f303xc.h"

/* Packet format: [start][size][type][payload][checksum][stop] */
#define SERIAL_START_BYTE      0x7EU
#define SERIAL_STOP_BYTE       0x7FU
#define SERIAL_MAX_PAYLOAD     64U
#define SERIAL_PACKET_OVERHEAD 5U
#define SERIAL_MAX_PACKET      (SERIAL_MAX_PAYLOAD + SERIAL_PACKET_OVERHEAD)

/* TX queue size for interrupt-driven transmission */
#define SERIAL_TX_BUFFER_SIZE  128U

typedef enum {
    SERIAL_MSG_DIRECTION = 1U,
} serial_msg_type_t;

typedef enum {
    DISPLAY_MODE_SERVO = 0U,
    DISPLAY_MODE_LED   = 1U
} display_mode_t;

// Describes one UART hardware instance: peripheral, IRQ, clock-enable register,
// clock-enable bit, TX/RX pins, and alternate function number.

typedef struct {
	USART_TypeDef *uart;
	IRQn_Type irqn;

	volatile uint32_t *rcc_en_reg;
	uint32_t rcc_en_mask;

	GPIO_TypeDef *tx_gpio;
	uint8_t tx_pin;

    GPIO_TypeDef *rx_gpio;
    uint8_t rx_pin;

    uint8_t af;
} serial_hw_t;

// for reading purpose
typedef struct serial_port serial_port_t;

// RX callback type
typedef void (*serial_rx_callback_t)(serial_port_t *port, const uint8_t *msg, uint8_t bytes_received);

// Ready-made hardware descriptors
extern const serial_hw_t SERIAL_HW_USART1_PC4_PC5;
extern const serial_hw_t SERIAL_HW_UART4_PC10_PC11;
extern serial_port_t serial_console;
extern serial_port_t serial_link;

// Basic UART setup
bool serial_init(serial_port_t *port, const serial_hw_t *hw, uint32_t peripheral_clock_hz, uint32_t baud);

// task a: sending / receiving bytes
bool serial_write_byte(serial_port_t *port, uint8_t byte);
bool serial_read_byte(serial_port_t *port, uint8_t *out_byte);
bool serial_send_bytes(serial_port_t *port, const uint8_t *data, size_t length);
bool serial_recv_bytes(serial_port_t *port, uint8_t *data, size_t length);

// task b: debugging string
bool serial_send_string(serial_port_t *port, const char *str);

// task c: send message send
bool serial_send_msg(serial_port_t *port, uint8_t msg_type, const void *payload, uint8_t payload_size);

// task e/f: interrupt-driven receive/transmit
void serial_set_receive_callback(serial_port_t *port, serial_rx_callback_t callback);
void serial_enable_rx_interrupt(serial_port_t *port);
void serial_disable_rx_interrupt(serial_port_t *port);
void serial_process_rx(serial_port_t *port);

// IRQ helper
void serial_irq_handler(serial_port_t *port);


#endif
