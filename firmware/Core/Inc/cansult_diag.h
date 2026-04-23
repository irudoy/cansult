#ifndef CANSULT_DIAG_H
#define CANSULT_DIAG_H

#include <stdint.h>

typedef struct {
  /* UART error counters (volatile — incremented from ISR or polled from SR) */
  volatile uint16_t uart_ore_count;
  volatile uint16_t uart_fe_count;
  volatile uint16_t uart_ne_count;

  /* System counters */
  uint16_t can_tx_fail_count;
  uint8_t can_recover_count;
  uint8_t dma_restart_count;
  uint8_t watchdog_timeout_count;
  uint16_t good_frame_count;
  /* Frames that assembled correctly but failed plausibility check
   * (e.g. RPM > 10000) — almost always a UART byte-shift from ORE/FE. */
  uint16_t implausible_frame_count;

  /* MCU internal temp sensor, °C * 10 (signed) */
  int16_t mcu_temp_c10;
  /* MCU temp at the moment of first FE, °C * 10 (INT16_MIN = no FE yet) */
  int16_t first_fe_temp_c10;
  /* State transition counters */
  uint8_t reconnect_count;      /* STARTUP entries (excl. first boot) */
} cansult_diag_t;

extern cansult_diag_t cansult_diag;

#endif
