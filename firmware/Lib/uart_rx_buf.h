#ifndef UART_RX_BUF_H
#define UART_RX_BUF_H

#include <stdint.h>
#include <stdbool.h>

#define UART_RX_BUF_SIZE 128  /* power of 2, ~133ms at 9600 baud */
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE - 1)

typedef struct {
    uint8_t  buf[UART_RX_BUF_SIZE];
    uint32_t head;  /* next write position */
    uint32_t tail;  /* next read position */
} uart_rx_buf_t;

void uart_rx_buf_init(uart_rx_buf_t *rb);
bool uart_rx_buf_push(uart_rx_buf_t *rb, uint8_t byte);
bool uart_rx_buf_pop(uart_rx_buf_t *rb, uint8_t *byte);
void uart_rx_buf_flush(uart_rx_buf_t *rb);
uint32_t uart_rx_buf_available(const uart_rx_buf_t *rb);

#endif
