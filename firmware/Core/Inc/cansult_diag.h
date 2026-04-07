#ifndef CANSULT_DIAG_H
#define CANSULT_DIAG_H

#include <stdint.h>

typedef struct {
  volatile uint8_t uart_ore_count;
  volatile uint8_t uart_fe_count;
  volatile uint8_t uart_ne_count;
  uint8_t can_tx_fail_count;
  uint8_t dma_restart_count;
  uint8_t watchdog_timeout_count;
  uint16_t good_frame_count;
} cansult_diag_t;

extern cansult_diag_t cansult_diag;

#endif
