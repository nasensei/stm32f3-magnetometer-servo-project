#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// format of our packet [start][size][type][payload][checksum][stop]
#define SERIAL_START_BYTE   0x7EU   // start of the packet
#define SERAL_STOP_BYTE     0x7FU   // end of the packet
#define SERIAL_MAX_PAYLOAD  64U     // max number of bytes in payload

typedef enum{
	SERIAL_MSG_DEBUG    = 1U,      // this packet is a debug msg
	SERIAL_MSG_HEADING  = 2U,      // this packet contains heading / compass data
	SERIAL_MSG_BUTTON   = 3U,      // this packet contains button state data
	SERIAL_MSG_CUSTOM   = 4U       // this is js a spare/ custom type for future extension
} serial_msg_type_t

// Basic UART setup
void serial_init(unit32_t peripheral_clock_hz, unit32_t baud);

// task a: sending/ receiving bytes
void serial_write_byte(unit8_t byte);
unit8_t serial_read_byte(void);
void serial_send_bytes(const unit8_t *data, size_t length);
void serial_recv_bytes(uint8_t *daa, size_t length);

// task b: debug string
void serial_send_string(const char *str);

// task c: framed message send
bool serial_send_msg(uint8_t msg_type, const void *payload, uint8_t payload_size);

#endif

