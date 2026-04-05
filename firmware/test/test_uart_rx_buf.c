#include "unity.h"
#include "uart_rx_buf.h"

static uart_rx_buf_t rb;

void setUp(void) { uart_rx_buf_init(&rb); }
void tearDown(void) {}

void test_empty_pop_returns_false(void) {
    uint8_t byte;
    TEST_ASSERT_FALSE(uart_rx_buf_pop(&rb, &byte));
}

void test_push_pop_single(void) {
    uint8_t byte;
    TEST_ASSERT_TRUE(uart_rx_buf_push(&rb, 0x42));
    TEST_ASSERT_TRUE(uart_rx_buf_pop(&rb, &byte));
    TEST_ASSERT_EQUAL_HEX8(0x42, byte);
    TEST_ASSERT_FALSE(uart_rx_buf_pop(&rb, &byte));
}

void test_push_pop_fifo_order(void) {
    uint8_t byte;
    uart_rx_buf_push(&rb, 0xAA);
    uart_rx_buf_push(&rb, 0xBB);
    uart_rx_buf_push(&rb, 0xCC);

    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xAA, byte);
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xBB, byte);
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xCC, byte);
}

void test_available(void) {
    TEST_ASSERT_EQUAL_UINT32(0, uart_rx_buf_available(&rb));
    uart_rx_buf_push(&rb, 0x01);
    uart_rx_buf_push(&rb, 0x02);
    TEST_ASSERT_EQUAL_UINT32(2, uart_rx_buf_available(&rb));

    uint8_t byte;
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_UINT32(1, uart_rx_buf_available(&rb));
}

void test_full_buffer_drops(void) {
    /* Fill to capacity (127 items, since 1 slot is sentinel) */
    for (int i = 0; i < UART_RX_BUF_SIZE - 1; i++) {
        TEST_ASSERT_TRUE(uart_rx_buf_push(&rb, (uint8_t)i));
    }
    TEST_ASSERT_EQUAL_UINT32(UART_RX_BUF_SIZE - 1, uart_rx_buf_available(&rb));

    /* Next push should fail (drop) */
    TEST_ASSERT_FALSE(uart_rx_buf_push(&rb, 0xFF));

    /* Existing data intact */
    uint8_t byte;
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0x00, byte);
}

void test_wraparound(void) {
    uint8_t byte;
    /* Push and pop to advance head/tail near the end of buffer */
    for (int i = 0; i < UART_RX_BUF_SIZE - 2; i++) {
        uart_rx_buf_push(&rb, (uint8_t)i);
        uart_rx_buf_pop(&rb, &byte);
    }

    /* Now push across the wrap boundary */
    uart_rx_buf_push(&rb, 0xDE);
    uart_rx_buf_push(&rb, 0xAD);
    uart_rx_buf_push(&rb, 0xBE);
    uart_rx_buf_push(&rb, 0xEF);

    TEST_ASSERT_EQUAL_UINT32(4, uart_rx_buf_available(&rb));

    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xDE, byte);
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xAD, byte);
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xBE, byte);
    uart_rx_buf_pop(&rb, &byte);
    TEST_ASSERT_EQUAL_HEX8(0xEF, byte);
}

void test_flush(void) {
    uart_rx_buf_push(&rb, 0x01);
    uart_rx_buf_push(&rb, 0x02);
    uart_rx_buf_flush(&rb);

    TEST_ASSERT_EQUAL_UINT32(0, uart_rx_buf_available(&rb));

    uint8_t byte;
    TEST_ASSERT_FALSE(uart_rx_buf_pop(&rb, &byte));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_empty_pop_returns_false);
    RUN_TEST(test_push_pop_single);
    RUN_TEST(test_push_pop_fifo_order);
    RUN_TEST(test_available);
    RUN_TEST(test_full_buffer_drops);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_flush);
    return UNITY_END();
}
