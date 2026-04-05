#include "uart_rx_buf.h"

void uart_rx_buf_init(uart_rx_buf_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool uart_rx_buf_push(uart_rx_buf_t *rb, uint8_t byte) {
    uint32_t next = (rb->head + 1) & UART_RX_BUF_MASK;
    if (next == rb->tail) return false;  /* full — drop */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

bool uart_rx_buf_pop(uart_rx_buf_t *rb, uint8_t *byte) {
    if (rb->head == rb->tail) return false;  /* empty */
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) & UART_RX_BUF_MASK;
    return true;
}

void uart_rx_buf_flush(uart_rx_buf_t *rb) {
    rb->tail = rb->head;
}

uint32_t uart_rx_buf_available(const uart_rx_buf_t *rb) {
    return (rb->head - rb->tail) & UART_RX_BUF_MASK;
}
