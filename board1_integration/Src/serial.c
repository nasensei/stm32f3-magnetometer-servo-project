#include "serial.h"
#include "stm32f303xc.h"

// maybe we can change these timeout values in the future
#define SERIAL_TX_TIMEOUT 1000000U
#define SERIAL_RX_TIMEOUT 1000000U

/// HELPPPPPP!!!!----------------------------------------------------------------
// ready-made hardware descriptors
const serial_hw_t SERIAL_HW_USART1_PC4_PC5 = {
    .uart        = USART1,
    .irqn        = USART1_IRQn,
    .rcc_en_reg  = &RCC->APB2ENR,
    .rcc_en_mask = RCC_APB2ENR_USART1EN,
    .tx_gpio     = GPIOC,
    .tx_pin      = 4U,
    .rx_gpio     = GPIOC,
    .rx_pin      = 5U,
    .af          = 7U
};

const serial_hw_t SERIAL_HW_UART4_PC10_PC11 = {
    .uart        = UART4,
    .irqn        = UART4_IRQn,
    .rcc_en_reg  = &RCC->APB1ENR,
    .rcc_en_mask = RCC_APB1ENR_UART4EN,
    .tx_gpio     = GPIOC,
    .tx_pin      = 10U,
    .rx_gpio     = GPIOC,
    .rx_pin      = 11U,
    .af          = 5U
};
/// HELPPPPPP!!!!----------------------------------------------------------------
/* this starts the actual internal structure of a serial port */

struct serial_port {
	const serial_hw_t *hw; // hardware
	serial_rx_callback_t rx_callback; // stores the function to call when a full valid msg is received
	// this is for double rx buffer, implementing part f
	volatile uint8_t rx_buffers[2][SERIAL_MAX_PACKET];
	volatile uint8_t rx_lengths[2];
	volatile bool rx_ready[2];
	volatile uint8_t rx_fill_index; // which of the two buffers ISR is currently filling
	volatile uint8_t rx_started; // tells whether a packet has started yet, meaning whether the start byte has already been seen
	//tx
    volatile uint8_t tx_buffer[SERIAL_TX_BUFFER_SIZE];
    volatile uint16_t tx_head; // is where the next byte will be written
    volatile uint16_t tx_tail; // is where the next byte will be read/sent
};

serial_port_t serial_console;
serial_port_t serial_link;

static void gpio_enable_clock(GPIO_TypeDef *gpio)
{
    if (gpio == GPIOA) {
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    } else if (gpio == GPIOB) {
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    } else if (gpio == GPIOC) {
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    } else if (gpio == GPIOD) {
        RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    } else if (gpio == GPIOE) {
        RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    } else if (gpio == GPIOF) {
        RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }
}

// configuring GPIO
static void gpio_set_alternate_function(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af) {
	uint32_t reg_index = pin / 8U; // AFR[0] for pins 0..7, AFR[1] for 8..15
	uint32_t shift = (pin % 8U) * 4U; // find bit position of this pin's 4-bit AF field
	gpio->AFR[reg_index] &= ~(0xFU << shift); // clear the old AF its for this pin
	gpio->AFR[reg_index] |=  (af << shift); // write new AF value into that field
}

static void gpio_config_uart_pin(GPIO_TypeDef *gpio, uint8_t pin, uint8_t af) {
    gpio->MODER &= ~(0x3U << (pin * 2U));
    gpio->MODER |=  (0x2U << (pin * 2U));   /* alternate function mode */

    gpio->OTYPER &= ~(1U << pin);           /* push-pull */
    gpio->OSPEEDR |= (0x3U << (pin * 2U));  /* high speed */
    gpio->PUPDR &= ~(0x3U << (pin * 2U));   /* no pull-up/pull-down */

    gpio_set_alternate_function(gpio, pin, af);
}
// end configuring gpio

static uint8_t serial_checksum(uint8_t msg_type, const uint8_t *payload, uint8_t payload_size)
{
    /* Simple 8-bit XOR checksum */
    uint8_t checksum = 0U;
    checksum ^= payload_size; // XOR in the payload size byte
    checksum ^= msg_type; //XOR the message type byte

    // loop through all payload bytes and XOR each one into the checksum
    for (uint8_t i = 0U; i < payload_size; i++) {
        checksum ^= payload[i];
    }

    return checksum;
}

static bool serial_validate_packet(const uint8_t *packet, uint8_t bytes_received)
{
    uint8_t payload_size;
    uint8_t msg_type;
    uint8_t received_checksum;
    uint8_t calculated_checksum;

    // reject if the pointer is NULL
    if (packet == NULL) {
        return false;
    }

    //reject if the packet is shorter than the minimum possible packet size
    if (bytes_received < SERIAL_PACKET_OVERHEAD) {
        return false;
    }

    // reject if the first byte is not the start byte
    if (packet[0] != SERIAL_START_BYTE) {
        return false;
    }

    // read the payload size from byte 1 and reject if it is too large
    payload_size = packet[1];
    if (payload_size > SERIAL_MAX_PAYLOAD) {
        return false;
    }

    // check that the total number of received bytes matches
    if (bytes_received != (uint8_t)(payload_size + SERIAL_PACKET_OVERHEAD)) {
        return false;
    }

    // check that the total stop byte is in the correct position
    if (packet[4U + payload_size] != SERIAL_STOP_BYTE) {
        return false;
    }

    /* read the msg type from byte 2 and read the checksum byte from the packet
     * then recompute what the checksum should be using the payload bytes
     */
    msg_type = packet[2];
    received_checksum = packet[3U + payload_size];
    calculated_checksum = serial_checksum(msg_type, &packet[3], payload_size);

    // reject if checksum does not match
    if (received_checksum != calculated_checksum) {
        return false;
    }

    return true;
}

// gets the next position in the circular TX buffer
static uint16_t serial_tx_next_index(uint16_t index) {
    index++;
    if (index >= SERIAL_TX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}
// check if the queue is empty
static bool serial_tx_queue_empty(const serial_port_t *port)
{
    return (port->tx_head == port->tx_tail);
}
// Queue is full if next head would collide with tail
static bool serial_tx_queue_full(const serial_port_t *port)
{
    return (serial_tx_next_index(port->tx_head) == port->tx_tail);
}

/*static bool serial_tx_enqueue(serial_port_t *port, uint8_t byte)
{
    uint32_t timeout = SERIAL_TX_TIMEOUT;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    while (timeout-- != 0U) {
        uint32_t primask = __get_PRIMASK();
        bool was_empty;

        __disable_irq();

        was_empty = serial_tx_queue_empty(port);

        if (!serial_tx_queue_full(port)) {
            port->tx_buffer[port->tx_head] = byte;
            port->tx_head = serial_tx_next_index(port->tx_head);


             * Prime the transmitter:
             * if queue was empty and hardware TX register is empty,
             * send the first queued byte immediately.

            if (was_empty && ((uart->ISR & USART_ISR_TXE) != 0U)) {
                uart->TDR = port->tx_buffer[port->tx_tail];
                port->tx_tail = serial_tx_next_index(port->tx_tail);
            }


             * Enable TXE interrupt so remaining queued bytes are sent by ISR.

            uart->CR1 |= USART_CR1_TXEIE;

            if (primask == 0U) {
                __enable_irq();
            }
            return true;
        }

        if (primask == 0U) {
            __enable_irq();
        }
    }

    return false;
}*/
// this adds one byte into the TX queue
static bool serial_tx_enqueue(serial_port_t *port, uint8_t byte)
{
    uint32_t timeout = SERIAL_TX_TIMEOUT;
    USART_TypeDef *uart;

    // reject invalid port
    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }
    // get the actual UART hardware register block
    uart = port->hw->uart;

    // keep trying until queue space becomes available or timeout expires
    while (timeout-- != 0U) {
        uint32_t primask = __get_PRIMASK(); // read current interrupt enable state,
        									// the primask here to rmb whether interrupts were enabled before you temporarily turned them off.
        __disable_irq(); // temporarily disable interrupts so queue update is atomic

        if (!serial_tx_queue_full(port)) { // this check if queue has room
            port->tx_buffer[port->tx_head] = byte; // store byte into tx buffer
            port->tx_head = serial_tx_next_index(port->tx_head); // advance the head pointer

            // enable TXE interrupt
            uart->CR1 |= USART_CR1_TXEIE;
            // restore interrupts if interrupts were previously enabled
            if (primask == 0U) {
                __enable_irq();
            }
            return true; // bye successfully enqueued
        }

        // if queue was full, restore interrupts and try again until timeout.
        if (primask == 0U) {
            __enable_irq();
        }
    }
    // if timeout expires, fail the operation
    return false;
}

// initialise one serial port instance
bool serial_init(serial_port_t *port, const serial_hw_t *hw, uint32_t peripheral_clock_hz, uint32_t baud) {
    if ((port == NULL) || (hw == NULL) || (baud == 0U)) {
        return false;
    }

    port->hw = hw;
    port->rx_callback = NULL;

    port->rx_lengths[0] = 0U;
    port->rx_lengths[1] = 0U;
    port->rx_ready[0] = false;
    port->rx_ready[1] = false;
    port->rx_fill_index = 0U;
    port->rx_started = false;

    port->tx_head = 0U;
    port->tx_tail = 0U;

    gpio_enable_clock(hw->tx_gpio);
    gpio_enable_clock(hw->rx_gpio);

    *(hw->rcc_en_reg) |= hw->rcc_en_mask;

    gpio_config_uart_pin(hw->tx_gpio, hw->tx_pin, hw->af);
    gpio_config_uart_pin(hw->rx_gpio, hw->rx_pin, hw->af);

    hw->uart->CR1 = 0U;
    hw->uart->CR2 = 0U;
    hw->uart->CR3 = 0U;

    hw->uart->BRR = (peripheral_clock_hz + (baud / 2U)) / baud;
    hw->uart->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    NVIC_ClearPendingIRQ(hw->irqn);
    NVIC_EnableIRQ(hw->irqn);

    return true;
}

// write one byte
bool serial_write_byte(serial_port_t *port, uint8_t byte)
{
    USART_TypeDef *uart;

    // reject invalid port
    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }
    // get UART pointer
    uart = port->hw->uart;
    // make sure UART is enabled and transmitter is enabled
    if ((uart->CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE)) {
        return false;
    }

    return serial_tx_enqueue(port, byte);
}

/*bool serial_write_byte(serial_port_t *port, uint8_t byte)
{
    uint32_t timeout = SERIAL_TX_TIMEOUT;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    if ((uart->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
        (USART_CR1_UE | USART_CR1_TE)) {
        return false;
    }

    while ((uart->ISR & USART_ISR_TXE) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    uart->TDR = byte;

    timeout = SERIAL_TX_TIMEOUT;
    while ((uart->ISR & USART_ISR_TC) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}*/
bool serial_read_byte(serial_port_t *port, uint8_t *out_byte)
{
    uint32_t timeout = SERIAL_RX_TIMEOUT;
    USART_TypeDef *uart;

    // reject invalid input
    if ((port == NULL) || (port->hw == NULL) || (out_byte == NULL)) {
        return false;
    }


    uart = port->hw->uart;

    // make sure uart receiver are enabled
    if ((uart->CR1 & (USART_CR1_UE | USART_CR1_RE)) !=
        (USART_CR1_UE | USART_CR1_RE)) {
        return false;
    }

    // wait until RXNE says a received byte us ready, unless timeout expires
    while ((uart->ISR & USART_ISR_RXNE) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    // read the byte from the receive data register and return success
    *out_byte = (uint8_t)(uart->RDR & 0xFFU);
    return true;
}

// if cant write byte = fail
bool serial_send_bytes(serial_port_t *port, const uint8_t *data, size_t length) {
    if ((data == NULL) && (length > 0U)) {
        return false;
    }

    for (size_t i = 0U; i < length; i++) {
        if (!serial_write_byte(port, data[i])) {
            return false;
        }
    }

    return true;
}

// if cant read byte = fail
bool serial_recv_bytes(serial_port_t *port, uint8_t *data, size_t length)
{
    if ((data == NULL) && (length > 0U)) {
        return false;
    }

    for (size_t i = 0U; i < length; i++) {
        if (!serial_read_byte(port, &data[i])) {
            return false;
        }
    }

    return true;
}

// this will be use to send the msg to console
bool serial_send_string(serial_port_t *port, const char *str)
{
    if (str == NULL) {
        return false;
    }

    while (*str != '\0') {
        if (!serial_write_byte(port, (uint8_t)(*str))) {
            return false;
        }
        str++;
    }

    return true;
}

// send the msg packet to another board
bool serial_send_msg(serial_port_t *port, uint8_t msg_type, const void *payload, uint8_t payload_size) {
    const uint8_t *payload_bytes = (const uint8_t *)payload;
    uint8_t packet[SERIAL_MAX_PACKET];
    uint8_t checksum;

    // reject oversize or invalid payload
    if ((payload_size > SERIAL_MAX_PAYLOAD) ||
        ((payload == NULL) && (payload_size > 0U))) {
        return false;
    }

    // compute checksum for this msg
    checksum = serial_checksum(msg_type, payload_bytes, payload_size);

    packet[0] = SERIAL_START_BYTE;
    packet[1] = payload_size;
    packet[2] = msg_type;

    // copy payload bytes into packet starting at index 3
    for (uint8_t i = 0U; i < payload_size; i++) {
        packet[3U + i] = payload_bytes[i];
    }

    packet[3U + payload_size] = checksum;
    packet[4U + payload_size] = SERIAL_STOP_BYTE;

    return serial_send_bytes(port, packet, (size_t)payload_size + SERIAL_PACKET_OVERHEAD);
}

// stores the callback function to use when a valid packet is received
void serial_set_receive_callback(serial_port_t *port, serial_rx_callback_t callback)
{
    if (port == NULL) {
        return;
    }

    port->rx_callback = callback;
}

bool serial_enable_rx_interrupt(serial_port_t *port)
{
    volatile uint8_t dummy;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    port->rx_lengths[0] = 0U;
    port->rx_lengths[1] = 0U;
    port->rx_ready[0] = false;
    port->rx_ready[1] = false;
    port->rx_fill_index = 0U;
    port->rx_started = false;

    // Flush any old unread bytes already sitting in the UART receive register
    while ((uart->ISR & USART_ISR_RXNE) != 0U) {
        dummy = (uint8_t)(uart->RDR & 0xFFU);
        (void)dummy;
    }

    uart->CR1 |= USART_CR1_RXNEIE;

    return ((uart->CR1 & USART_CR1_RXNEIE) != 0U);
}

void serial_disable_rx_interrupt(serial_port_t *port)
{
    if ((port == NULL) || (port->hw == NULL)) {
        return;
    }

    port->hw->uart->CR1 &= ~USART_CR1_RXNEIE;
}

/* This is the function the main loop calls to process completed RX packets
 * the UART interrupt handler collects incoming bytes and fills one of the RX buffer
 * once it sees a full packet, it marks that buffer as ready
 * serial_process_rx() is the function that checks those ready buffers,
 * validates the packet, calls the callback then clears the buffer
 */
void serial_process_rx(serial_port_t *port) {
    uint8_t idx;

    if (port == NULL) {
        return;
    }

    if (port->rx_ready[0]) {
        idx = 0U;
    } else if (port->rx_ready[1]) {
        idx = 1U;
    } else {
        return;
    }

    /* validate the finished packet
     * if valid, it will go to the second if:
     * the program will call the user callback
    */
    if (serial_validate_packet((const uint8_t *)port->rx_buffers[idx],
                               port->rx_lengths[idx])) {
        if (port->rx_callback != NULL) {
            port->rx_callback(port,
                              (const uint8_t *)port->rx_buffers[idx],
                              port->rx_lengths[idx]);
        }
    }
    // after processing, clear this buffer's length and ready flag
    port->rx_lengths[idx] = 0U;
    port->rx_ready[idx] = false;

    // if no packet is currently being received, but the current fill buffer
    // is actually still marked ready, then switch the fill index
    if (!port->rx_started && port->rx_ready[port->rx_fill_index]) {
        port->rx_fill_index = idx;
    }
}

/* RX part: grab incoming bytes from UART and build packets in the RX buffers
 * TX part: take queued bytes from the TX buffer and send them out
 */
void serial_irq_handler(serial_port_t *port)
{
    USART_TypeDef *uart;
    uint32_t isr;

    if ((port == NULL) || (port->hw == NULL)) {
        return;
    }

    uart = port->hw->uart;
    isr = uart->ISR;

    /* ---------- RX handling ---------- */
    // if RXNE is set, there is a received byte waiting
    if ((isr & USART_ISR_RXNE) != 0U) {
        uint8_t byte = (uint8_t)(uart->RDR & 0xFFU); // this reads the received byte from the UART receive data register
        /* double buffering so there are two RX buffers
         * fill is the buffer currently being filled
         * other is the other one
         */
        uint8_t fill = port->rx_fill_index;
        uint8_t other = (uint8_t)(fill ^ 1U);

        // if a packet has not started yet
        if (!port->rx_started) {
        	// if the received byte is the start marker, then this is the beginning of a new packet
            if (byte == SERIAL_START_BYTE) { // make sure that this is the start byte
            	/* is the buffer was about to fill still marked as holding an unprocessed complete packet
            	 * if yes, that buffer should not be overwritten
            	 */
                if (port->rx_ready[fill]) {
                	// check if the other buffer is busy
                    if (port->rx_ready[other]) {
                    	// if both are busy, drop this incoming packet
                        return;
                    }
                    // switch to the other buffer
                    port->rx_fill_index = other;
                    fill = other;
                }
                // start receiving the new packet
                port->rx_lengths[fill] = 0U; // clear the length for the selected buffer
                port->rx_started = true; // mark that a packet is now officially in progress
            } else { // if this is not a start byte, ignore it
                return;
            }
        }

        fill = port->rx_fill_index; //refresh fill index
        /* this checks whether the packet has grown too large
         * if it has:
         * reset the buffer length
         * cancel the packet
         * stop reception of that packet
         * return
         */
        if (port->rx_lengths[fill] >= SERIAL_MAX_PACKET) {
            port->rx_lengths[fill] = 0U;
            port->rx_started = false;
            return;
        }

        // stores the received byte into the current buffer at the current length index, then increments the length
        port->rx_buffers[fill][port->rx_lengths[fill]++] = byte;
        // if its at stop byte
        if (byte == SERIAL_STOP_BYTE) {
            port->rx_ready[fill] = true; // this RX buffer now contains a complete packet
            port->rx_started = false; // packet reception is no longer in progress

            other = (uint8_t)(fill ^ 1U); // get the alternate buffer index

            /*
             * if the other buffer is not busy, switch future RX into that other buffer and clear its length
             * one buffer now holds a completed packet
             * the other buffer becomes the next target for new incoming bytes
             */
            if (!port->rx_ready[other]) {
                port->rx_fill_index = other;
                port->rx_lengths[other] = 0U;
            }
        }
    }

    /* ---------- TX handling ---------- */

    /* this checks two things:
     * TXEIE is enabled meaning TX-empty interrupts are turned on
     * TXE is set meaning the transmit data register is empty and ready for another byte
     */
    if (((uart->CR1 & USART_CR1_TXEIE) != 0U) &&
        ((uart->ISR & USART_ISR_TXE) != 0U)) {
    	// if TX queue is not empty, if yes send the next one
        if (!serial_tx_queue_empty(port)) {
            uart->TDR = port->tx_buffer[port->tx_tail]; // write next byte to TDR, transmit data register
            port->tx_tail = serial_tx_next_index(port->tx_tail); // advance the tail pointer
        } else {
        	// if the queue is empty
        	// disable the TXE interrupt
            uart->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void USART1_EXTI25_IRQHandler(void) {
    serial_irq_handler(&serial_console);
}

void UART4_EXTI34_IRQHandler(void) {
    serial_irq_handler(&serial_link);
}
