#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Packet format: [start][size][type][payload][checksum][stop] */
#define SERIAL_START_BYTE      0x7EU
#define SERIAL_STOP_BYTE       0x7FU
#define SERIAL_MAX_PAYLOAD     64U
#define SERIAL_PACKET_OVERHEAD 5U
#define SERIAL_MAX_PACKET      (SERIAL_MAX_PAYLOAD + SERIAL_PACKET_OVERHEAD)

typedef enum {
    SERIAL_MSG_DEBUG   = 1U,
    SERIAL_MSG_HEADING = 2U,
    SERIAL_MSG_BUTTON  = 3U,
    SERIAL_MSG_CUSTOM  = 4U
} serial_msg_type_t;

typedef void (*serial_rx_callback_t)(const uint8_t *msg, uint8_t bytes_received);

/* Basic UART setup */
void serial_init(uint32_t peripheral_clock_hz, uint32_t baud);

/* Task a: sending / receiving bytes */
bool serial_write_byte(uint8_t byte);
bool serial_read_byte(uint8_t *out_byte);
bool serial_send_bytes(const uint8_t *data, size_t length);
bool serial_recv_bytes(uint8_t *data, size_t length);

/* Task b: debug string */
bool serial_send_string(const char *str);

/* Task c: framed message send */
bool serial_send_msg(uint8_t msg_type, const void *payload, uint8_t payload_size);

/* Task d: polling receive + callback
void serial_set_receive_callback(serial_rx_callback_t callback);
bool serial_receive_msg(void); */

/* Task e: interrupt-driven receive */
void serial_set_receive_callback(serial_rx_callback_t callback);
void serial_enable_rx_interrupt(void);
void serial_disable_rx_interrupt(void);
void serial_process_rx(void);

#endif
