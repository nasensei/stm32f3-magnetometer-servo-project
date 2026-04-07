#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Packet format: [start][size][type][payload][checksum][stop] */
#define SERIAL_START_BYTE   0x7EU
#define SERIAL_STOP_BYTE    0x7FU
#define SERIAL_MAX_PAYLOAD  64U

typedef enum {
    SERIAL_MSG_DEBUG   = 1U,
    SERIAL_MSG_HEADING = 2U,
    SERIAL_MSG_BUTTON  = 3U,
    SERIAL_MSG_CUSTOM  = 4U
} serial_msg_type_t;

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

#endif
