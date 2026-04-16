#include "serial.h"
#include "stm32f303xc.h"

#define USART1_TX_PIN   4U   /* PC4 -> USART1_TX */
#define USART1_RX_PIN   5U   /* PC5 -> USART1_RX */
#define USART_AF_NUM    7U   /* AF7 for USART1 */
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

struct serial_port {
	const serial_hw_t *hw;
	serial_rx_callback_t rx_callback;
	// this is for double rx buffer, implementing part f
	volatile uint8_t rx_buffers[2][SERIAL_MAX_PACKET];
	volatile uint8_t rx_lengths[2];
	volatile bool rx_ready[2];
	volatile uint8_t rx_fill_index;
	volatile uint8_t rx_started;
	//tx
    volatile uint8_t tx_buffer[SERIAL_TX_BUFFER_SIZE];
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;
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

static bool serial_validate_packet(const uint8_t *packet, uint8_t bytes_received)
{
    uint8_t payload_size;
    uint8_t msg_type;
    uint8_t received_checksum;
    uint8_t calculated_checksum;

    if (packet == NULL) {
        return false;
    }

    if (bytes_received < SERIAL_PACKET_OVERHEAD) {
        return false;
    }

    if (packet[0] != SERIAL_START_BYTE) {
        return false;
    }

    payload_size = packet[1];
    if (payload_size > SERIAL_MAX_PAYLOAD) {
        return false;
    }

    if (bytes_received != (uint8_t)(payload_size + SERIAL_PACKET_OVERHEAD)) {
        return false;
    }

    if (packet[4U + payload_size] != SERIAL_STOP_BYTE) {
        return false;
    }

    msg_type = packet[2];
    received_checksum = packet[3U + payload_size];
    calculated_checksum = serial_checksum(msg_type, &packet[3], payload_size);

    if (received_checksum != calculated_checksum) {
        return false;
    }

    return true;
}

static uint16_t serial_tx_next_index(uint16_t index) {
    index++;
    if (index >= SERIAL_TX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}
static bool serial_tx_queue_empty(const serial_port_t *port)
{
    return (port->tx_head == port->tx_tail);
}

static bool serial_tx_queue_full(const serial_port_t *port)
{
    return (serial_tx_next_index(port->tx_head) == port->tx_tail);
}

static bool serial_tx_enqueue(serial_port_t *port, uint8_t byte)
{
    uint32_t timeout = SERIAL_TX_TIMEOUT;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    while (timeout-- != 0U) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (!serial_tx_queue_full(port)) {
            port->tx_buffer[port->tx_head] = byte;
            port->tx_head = serial_tx_next_index(port->tx_head);

            /* Kick TX interrupt so ISR starts draining the queue */
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
}

bool serial_init(serial_port_t *port,
                 const serial_hw_t *hw,
                 uint32_t peripheral_clock_hz,
                 uint32_t baud)
{
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

bool serial_write_byte(serial_port_t *port, uint8_t byte)
{
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    if ((uart->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
        (USART_CR1_UE | USART_CR1_TE)) {
        return false;
    }

    return serial_tx_enqueue(port, byte);
}

bool serial_read_byte(serial_port_t *port, uint8_t *out_byte)
{
    uint32_t timeout = SERIAL_RX_TIMEOUT;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL) || (out_byte == NULL)) {
        return false;
    }

    uart = port->hw->uart;

    if ((uart->CR1 & (USART_CR1_UE | USART_CR1_RE)) !=
        (USART_CR1_UE | USART_CR1_RE)) {
        return false;
    }

    while ((uart->ISR & USART_ISR_RXNE) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    *out_byte = (uint8_t)(uart->RDR & 0xFFU);
    return true;
}

bool serial_send_bytes(serial_port_t *port, const uint8_t *data, size_t length)
{
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

bool serial_send_msg(serial_port_t *port,
                     uint8_t msg_type,
                     const void *payload,
                     uint8_t payload_size)
{
    const uint8_t *payload_bytes = (const uint8_t *)payload;
    uint8_t packet[SERIAL_MAX_PACKET];
    uint8_t checksum;

    if ((payload_size > SERIAL_MAX_PAYLOAD) ||
        ((payload == NULL) && (payload_size > 0U))) {
        return false;
    }

    checksum = serial_checksum(msg_type, payload_bytes, payload_size);

    packet[0] = SERIAL_START_BYTE;
    packet[1] = payload_size;
    packet[2] = msg_type;

    for (uint8_t i = 0U; i < payload_size; i++) {
        packet[3U + i] = payload_bytes[i];
    }

    packet[3U + payload_size] = checksum;
    packet[4U + payload_size] = SERIAL_STOP_BYTE;

    return serial_send_bytes(port, packet, (size_t)payload_size + SERIAL_PACKET_OVERHEAD);
}

void serial_set_receive_callback(serial_port_t *port, serial_rx_callback_t callback)
{
    if (port == NULL) {
        return;
    }

    port->rx_callback = callback;
}

void serial_enable_rx_interrupt(serial_port_t *port)
{
    volatile uint8_t dummy;
    USART_TypeDef *uart;

    if ((port == NULL) || (port->hw == NULL)) {
        return;
    }

    uart = port->hw->uart;

    port->rx_lengths[0] = 0U;
    port->rx_lengths[1] = 0U;
    port->rx_ready[0] = false;
    port->rx_ready[1] = false;
    port->rx_fill_index = 0U;
    port->rx_started = false;

    while ((uart->ISR & USART_ISR_RXNE) != 0U) {
        dummy = (uint8_t)(uart->RDR & 0xFFU);
        (void)dummy;
    }

    uart->CR1 |= USART_CR1_RXNEIE;
}

void serial_disable_rx_interrupt(serial_port_t *port)
{
    if ((port == NULL) || (port->hw == NULL)) {
        return;
    }

    port->hw->uart->CR1 &= ~USART_CR1_RXNEIE;
}

void serial_process_rx(serial_port_t *port)
{
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

    if (serial_validate_packet((const uint8_t *)port->rx_buffers[idx],
                               port->rx_lengths[idx])) {
        if (port->rx_callback != NULL) {
            port->rx_callback(port,
                              (const uint8_t *)port->rx_buffers[idx],
                              port->rx_lengths[idx]);
        }
    }

    port->rx_lengths[idx] = 0U;
    port->rx_ready[idx] = false;

    if (!port->rx_started && port->rx_ready[port->rx_fill_index]) {
        port->rx_fill_index = idx;
    }
}

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
    if ((isr & USART_ISR_RXNE) != 0U) {
        uint8_t byte = (uint8_t)(uart->RDR & 0xFFU);
        uint8_t fill = port->rx_fill_index;
        uint8_t other = (uint8_t)(fill ^ 1U);

        if (!port->rx_started) {
            if (byte == SERIAL_START_BYTE) {
                if (port->rx_ready[fill]) {
                    if (port->rx_ready[other]) {
                        return;
                    }
                    port->rx_fill_index = other;
                    fill = other;
                }

                port->rx_lengths[fill] = 0U;
                port->rx_started = true;
            } else {
                return;
            }
        }

        fill = port->rx_fill_index;

        if (port->rx_lengths[fill] >= SERIAL_MAX_PACKET) {
            port->rx_lengths[fill] = 0U;
            port->rx_started = false;
            return;
        }

        port->rx_buffers[fill][port->rx_lengths[fill]++] = byte;

        if (byte == SERIAL_STOP_BYTE) {
            port->rx_ready[fill] = true;
            port->rx_started = false;

            other = (uint8_t)(fill ^ 1U);

            if (!port->rx_ready[other]) {
                port->rx_fill_index = other;
                port->rx_lengths[other] = 0U;
            }
        }
    }

    /* ---------- TX handling ---------- */
    if (((uart->CR1 & USART_CR1_TXEIE) != 0U) &&
        ((uart->ISR & USART_ISR_TXE) != 0U)) {

        if (!serial_tx_queue_empty(port)) {
            uart->TDR = port->tx_buffer[port->tx_tail];
            port->tx_tail = serial_tx_next_index(port->tx_tail);
        } else {
            uart->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void USART1_IRQHandler(void)
{
    serial_irq_handler(&serial_console);
}

void UART4_IRQHandler(void)
{
    serial_irq_handler(&serial_link);
}
