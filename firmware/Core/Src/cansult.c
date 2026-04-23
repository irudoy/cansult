#include "cansult.h"
#include "cansult_diag.h"
#include "uart_rx_buf.h"
#include "consult_parser.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan;
extern ADC_HandleTypeDef hadc1;

/* --- Diagnostics --- */
cansult_diag_t cansult_diag;

/* --- DMA RX --- */
#define DMA_RX_BUF_SIZE 128
static uint8_t dmaRxBuf[DMA_RX_BUF_SIZE];
static uint32_t dmaRxReadPos = 0;
static volatile bool uartIdleFlag = false;

/* --- Ring buffer (between DMA and parser) --- */
static uart_rx_buf_t rxBuf;

/* --- Protocol parser --- */
static consult_parser_t parser;

/* --- Debug stream --- */
static volatile bool debugStream = false;
static uint8_t dbgRxBuf[8];
static uint8_t dbgRxLen = 0;

/* --- State --- */
static bool forceStopStream = false;
static uint8_t heartbeat = 0;
static uint32_t time;
static uint32_t tempTime;
static uint32_t lastFrameSentTime;
static uint32_t lastDiagTime;
static cansult_mode_t mode = CANSULT_MODE_STREAM;

/* --- DMA drain: copy new bytes from DMA circular buffer to ring buffer --- */
static void drainDmaToRingBuf(void) {
  uint32_t dmaWritePos = DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  while (dmaRxReadPos != dmaWritePos) {
    uart_rx_buf_push(&rxBuf, dmaRxBuf[dmaRxReadPos]);
    dmaRxReadPos = (dmaRxReadPos + 1) % DMA_RX_BUF_SIZE;
  }
}

/* --- Non-blocking byte read --- */
static bool readEcuByte(uint8_t *byte) {
  if (uartIdleFlag) {
    uartIdleFlag = false;
    drainDmaToRingBuf();
  }
  drainDmaToRingBuf();
  return uart_rx_buf_pop(&rxBuf, byte);
}

static void dbgSendTx(const uint8_t *buf, uint16_t len);

/* --- UART TX (short timeout) --- */
static HAL_StatusTypeDef writeEcu(uint8_t b) {
  dbgSendTx(&b, 1);
  return HAL_UART_Transmit(&huart1, &b, 1, 5);
}

/* --- Batch UART TX --- */
static HAL_StatusTypeDef writeEcuBuf(const uint8_t *buf, uint16_t len) {
  dbgSendTx(buf, len);
  return HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, len * 2 + 5);
}

/* --- Stop stream: send stop, wait for ACK, flush --- */
static void stopStream(void) {
  consult_parser_set_state(&parser, CONSULT_STATE_IDLE);
  consult_parser_reset_frame(&parser);

  /* Send stop multiple times — ECU may miss it mid-stream */
  for (int i = 0; i < 3; i++) {
    writeEcu(CANSULT_ECU_COMMAND_STOP_STREAM);
  }

  uint32_t stopTime = HAL_GetTick();
  while (HAL_GetTick() - stopTime < 200) {
    uint8_t byte;
    if (readEcuByte(&byte)) {
      if (byte == 0xCF) break;
    }
  }
  uart_rx_buf_flush(&rxBuf);
}

/* --- Init ECU --- */
static void initECU(void) {
  stopStream();
  consult_parser_set_state(&parser, CONSULT_STATE_INITIALIZING);
  static const uint8_t initCmd[] = {
    0xFF, 0xFF, CANSULT_ECU_COMMAND_INIT,
    0xFF, 0xFF, CANSULT_ECU_COMMAND_INIT
  };
  writeEcuBuf(initCmd, sizeof(initCmd));
  tempTime = time;
}

/* --- Post-init: request ECU info --- */
static void postInit(void) {
  stopStream();
  consult_parser_set_state(&parser, CONSULT_STATE_WAITING_ECU_RESPONSE);
  tempTime = HAL_GetTick();
  writeEcu(CANSULT_ECU_COMMAND_ECU_INFO);
  writeEcu(CANSULT_ECU_COMMAND_TERM);
}

/* --- Read DTC --- */
static void readFaultCodes(void) {
  forceStopStream = true;
  stopStream();
  consult_parser_set_state(&parser, CONSULT_STATE_WAITING_ECU_RESPONSE);
  tempTime = HAL_GetTick();
  writeEcu(CANSULT_ECU_COMMAND_SELF_DIAG);
  writeEcu(CANSULT_ECU_COMMAND_TERM);
}

/* --- Resume main stream after DTC read --- */
static void resumeMainStream(void) {
  stopStream();
  forceStopStream = false;
}

/* --- Request streaming (batch TX) --- */
static void requestStreaming(void) {
  stopStream();
  consult_parser_set_state(&parser, CONSULT_STATE_STREAMING);
  tempTime = HAL_GetTick();

  static const uint8_t streamCmd[] = {
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_BATTERY_VOLTAGE,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_COOLANT_TEMP,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_IGNITION_TIMING,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_LEFT_O2,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_TPS,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_AAC_VALVE,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_LEFT_AF_ALPHA,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_LEFT_AF_ALPHA_SELFLEARN,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_VEHICLE_SPEED,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_TACH_MSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_TACH_LSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_INJ_TIME_MSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_INJ_TIME_LSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_LEFT_MAF_MSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_LEFT_MAF_LSB,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_BIT_1,
    CANSULT_ECU_COMMAND_READ_REGISTER, CANSULT_ECU_REGISTER_BIT_2,
    CANSULT_ECU_COMMAND_TERM
  };
  writeEcuBuf(streamCmd, sizeof(streamCmd));
}

/* --- CAN TX helper with mailbox check --- */
static void canTx(uint16_t id, uint8_t *data, uint8_t dlc) {
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = id;
  txHeader.DLC = dlc;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0 ||
      HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK) {
    cansult_diag.can_tx_fail_count++;
  }
}

/* --- Debug stream: flush buffered RX bytes as CAN frame --- */
static void dbgFlushRx(void) {
  if (dbgRxLen > 0) {
    canTx(CANSULT_CAN_ID_DBG_RX, dbgRxBuf, dbgRxLen);
    dbgRxLen = 0;
  }
}

/* --- Debug stream: send TX bytes as CAN frame --- */
static void dbgSendTx(const uint8_t *buf, uint16_t len) {
  if (!debugStream) return;
  while (len > 0) {
    uint8_t chunk = (len > 8) ? 8 : (uint8_t)len;
    canTx(CANSULT_CAN_ID_DBG_TX, (uint8_t *)buf, chunk);
    buf += chunk;
    len -= chunk;
  }
}

/* --- Read MCU internal temperature sensor, returns °C * 10 --- */
static int16_t readMcuTempC10(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return INT16_MIN;
  }
  uint32_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  /* Vref = 3.3 V, 12-bit ADC. Datasheet: V25 = 1.43 V, Avg_Slope = 4.3 mV/°C
     T(°C) = (V25 - Vsense)/Slope + 25
     mV = raw * 3300 / 4095
     T*10 = (14300 - mV*10)/43 + 250 */
  int32_t mv_x10 = (int32_t)raw * 33000 / 4095;
  int32_t t_c10 = (14300 - mv_x10) / 43 + 250;
  return (int16_t)t_c10;
}

/* --- Diagnostic frames 0x665 + 0x66B at 1Hz --- */
static uint16_t lastOreSnap, lastImplausibleSnap, lastCanSnap;

static uint8_t deltaSat(uint16_t prev, uint16_t curr) {
  uint16_t d = curr - prev;
  return d > 255u ? 255u : (uint8_t)d;
}

static void sendDiagFrame(void) {
  if (time - lastDiagTime < 1000ul) return;
  lastDiagTime = time;

  /* 0x665: bytes 1-3 are per-second rates (saturating at 255),
     bytes 4-6 are monotonic u8 counters, byte 7 is ms since last frame /4.
     FE+NE rate isn't emitted here — derive it from the u16 FE/NE counters
     in 0x66B by taking the delta between two consecutive diag samples. */
  uint8_t data[8];
  data[0] = parser.state;
  data[1] = deltaSat(lastOreSnap, cansult_diag.uart_ore_count);
  data[2] = deltaSat(lastImplausibleSnap, cansult_diag.implausible_frame_count);
  data[3] = deltaSat(lastCanSnap, cansult_diag.can_tx_fail_count);
  data[4] = cansult_diag.dma_restart_count;
  data[5] = cansult_diag.watchdog_timeout_count;
  data[6] = cansult_diag.reconnect_count;
  data[7] = (uint8_t)((time - tempTime) >> 2);
  canTx(CANSULT_CAN_ID_DIAG, data, 8);
  lastOreSnap = cansult_diag.uart_ore_count;
  lastImplausibleSnap = cansult_diag.implausible_frame_count;
  lastCanSnap = cansult_diag.can_tx_fail_count;

  /* Update current MCU temperature (signed, °C*10) */
  cansult_diag.mcu_temp_c10 = readMcuTempC10();

  /* 0x66B: extended diag — full 16-bit FE/NE, current MCU temp,
     CAN recover count, current HAL CAN state. */
  uint8_t d2[8];
  d2[0] = (uint8_t)(cansult_diag.uart_fe_count >> 8);    /* FE high */
  d2[1] = (uint8_t)(cansult_diag.uart_fe_count);         /* FE low  */
  d2[2] = (uint8_t)(cansult_diag.uart_ne_count >> 8);    /* NE high */
  d2[3] = (uint8_t)(cansult_diag.uart_ne_count);         /* NE low  */
  d2[4] = (uint8_t)((uint16_t)cansult_diag.mcu_temp_c10 >> 8);
  d2[5] = (uint8_t)((uint16_t)cansult_diag.mcu_temp_c10);
  d2[6] = cansult_diag.can_recover_count;
  /* byte 7: bit 7 = mode (1 = ADAPTER), bits 0-6 = hcan.State (≤5). */
  d2[7] = (uint8_t)((mode == CANSULT_MODE_ADAPTER ? 0x80u : 0u) |
                    ((uint8_t)hcan.State & 0x7Fu));
  canTx(CANSULT_CAN_ID_DIAG2, d2, 8);
}

/* --- Process all buffered ECU bytes through parser --- */
static void processEcuBytes(void) {
  uint8_t byte;
  while (readEcuByte(&byte)) {
    if (debugStream) {
      dbgRxBuf[dbgRxLen++] = byte;
      if (dbgRxLen == 8) dbgFlushRx();
    }
    consult_event_t ev = consult_parser_feed(&parser, byte);

    /* React to parser events */
    switch (ev) {
    case CONSULT_EVENT_INIT_OK:
      /* parser already moved to POST_INIT */
      break;
    case CONSULT_EVENT_ECU_INFO_READY:
      readFaultCodes();
      break;
    case CONSULT_EVENT_STREAM_FRAME:
      /* Reject impossible values (RPM>10000, Speed>250 etc). These come
       * from UART byte-shifts (no CRC in Consult stream) and would send
       * 745663 rpm spikes downstream. On failure the parser rolls back
       * to last-good data so CAN TX repeats the previous frame. */
      if (consult_parser_validate_stream(&parser)) {
        tempTime = time;  /* watchdog: got data */
        cansult_diag.good_frame_count++;
      } else {
        cansult_diag.implausible_frame_count++;
      }
      break;
    case CONSULT_EVENT_FAULT_CODES_READY:
      resumeMainStream();
      break;
    case CONSULT_EVENT_CLEAR_CODES_OK:
      readFaultCodes();
      break;
    case CONSULT_EVENT_NONE:
      /* Update watchdog on any byte during streaming */
      if (parser.state == CONSULT_STATE_STREAMING && parser.reading_frame)
        tempTime = time;
      break;
    }
  }
  if (debugStream) dbgFlushRx();
}

/* --- Send CAN frames with current data --- */
static void sendCanFrames(void) {
  uint8_t txData[8];

  for (int i = 0; i < 8; i++) txData[i] = parser.data[i];
  canTx(CANSULT_CAN_ID_1, txData, 8);

  for (int i = 0; i < 7; i++) txData[i] = parser.data[8 + i];
  txData[7] = 0xFF;
  canTx(CANSULT_CAN_ID_2, txData, 8);

  txData[0] = parser.data[15];
  txData[1] = parser.data[16];
  txData[2] = 0xFF;
  txData[3] = 0xFF;
  txData[4] = 0xFF;
  txData[5] = 0xFF;
  txData[6] = parser.fault_codes[0];
  txData[7] = heartbeat;
  canTx(CANSULT_CAN_ID_3, txData, 8);

  lastFrameSentTime = time;
  heartbeat++;
}

/* --- State machine router --- */
static void route(void) {
  static uint8_t prevRouteState = CONSULT_STATE_STARTUP;
  /* Count transitions *into* STARTUP (skip the initial boot-time entry) */
  if (parser.state != prevRouteState) {
    if (parser.state == CONSULT_STATE_STARTUP && time > 1000) {
      cansult_diag.reconnect_count++;
    }
    prevRouteState = parser.state;
  }
  switch (parser.state) {
  case CONSULT_STATE_STARTUP:
    initECU();
    break;
  case CONSULT_STATE_INITIALIZING:
    if (time - tempTime > 3000ul)
      consult_parser_set_state(&parser, CONSULT_STATE_STARTUP);
    break;
  case CONSULT_STATE_POST_INIT:
    postInit();
    break;
  case CONSULT_STATE_WAITING_ECU_RESPONSE:
    if (time - tempTime > 2000ul) {
      /* If fault codes request timed out, proceed to streaming anyway */
      forceStopStream = false;
      consult_parser_reset_frame(&parser);
      consult_parser_set_state(&parser, CONSULT_STATE_IDLE);
    }
    break;
  case CONSULT_STATE_IDLE:
    if (!forceStopStream)
      requestStreaming();
    break;
  case CONSULT_STATE_STREAMING:
    if (time - tempTime > 500ul) {
      cansult_diag.watchdog_timeout_count++;
      stopStream();
      consult_parser_set_state(&parser, CONSULT_STATE_STARTUP);
    }
    if (time - lastFrameSentTime > CANSULT_DATA_RATE)
      sendCanFrames();
    break;
  }
}

/* --- IDLE line callback (called from ISR) --- */
void cansult_uart_idle_callback(void) {
  uartIdleFlag = true;
}

/* --- Public API --- */

/* --- CAN filter: accept 0x66F (debug cmd) + 0x66D (mode cmd) on FIFO0
   in 16-bit ID list mode (4 ID slots total, duplicate each for clarity). */
static void configCanFilter(void) {
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh = CANSULT_CAN_ID_DBG_CMD << 5;
  filter.FilterIdLow = CANSULT_CAN_ID_DBG_CMD << 5;
  filter.FilterMaskIdHigh = CANSULT_CAN_ID_MODE_CMD << 5;
  filter.FilterMaskIdLow = CANSULT_CAN_ID_MODE_CMD << 5;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &filter);
}

/* --- Reconfigure PA9 (USART1_TX) between AF push-pull and GPIO input.
   In adapter mode we release PA9 to high-Z so the BT module owns MCU_TX;
   PA10 (RX) stays an input either way, so no reconfig needed there. */
static void setUart1TxPin(uint32_t gpioMode, uint32_t speed) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_9;
  g.Mode = gpioMode;
  g.Pull = GPIO_NOPULL;
  g.Speed = speed;
  HAL_GPIO_Init(GPIOA, &g);
}

static void enterAdapterMode(void) {
  if (mode == CANSULT_MODE_ADAPTER) return;
  /* Tell ECU to halt the MCU-initiated stream before we yield the line —
     otherwise it keeps blasting telemetry and confuses PC-side Consult
     software talking through BT. */
  stopStream();
  /* Release MCU_TX to high-Z; BT module will drive it from now on. */
  setUart1TxPin(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW);
  /* Power up BT module (BC417 expansion board). */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  /* Reset parser/buffers so on exit we re-handshake with ECU cleanly. */
  consult_parser_init(&parser);
  uart_rx_buf_flush(&rxBuf);
  mode = CANSULT_MODE_ADAPTER;
}

static void exitAdapterMode(void) {
  if (mode == CANSULT_MODE_STREAM) return;
  /* Power down BT module first so it stops driving the shared line. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  /* Reclaim MCU_TX. */
  setUart1TxPin(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH);
  /* Skip whatever the bus put into the DMA ring during pass-through. */
  dmaRxReadPos = DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  uart_rx_buf_flush(&rxBuf);
  consult_parser_init(&parser);
  tempTime = HAL_GetTick();  /* arm watchdog */
  mode = CANSULT_MODE_STREAM;
}

/* --- Recover CAN peripheral from bus-off / RESET / ERROR state.
   Observed 2026-04-15: brownout cleared CAN registers while MCU core
   survived; firmware kept running but TX silently stopped. */
static void recoverCan(void) {
  (void)HAL_CAN_Stop(&hcan);
  (void)HAL_CAN_DeInit(&hcan);
  (void)HAL_CAN_Init(&hcan);
  configCanFilter();
  (void)HAL_CAN_Start(&hcan);
  if (cansult_diag.can_recover_count < 255)
    cansult_diag.can_recover_count++;
}

void cansult_init(void) {
  /* BT_EN (PA8) output, initially low — BT module powered off in STREAM mode.
     Not configured via CubeMX; set up here so regeneration doesn't fight us. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  {
    GPIO_InitTypeDef bt = {0};
    bt.Pin = GPIO_PIN_8;
    bt.Mode = GPIO_MODE_OUTPUT_PP;
    bt.Pull = GPIO_NOPULL;
    bt.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &bt);
  }

  configCanFilter();
  HAL_CAN_Start(&hcan);
  uart_rx_buf_init(&rxBuf);
  consult_parser_init(&parser);
  cansult_diag.first_fe_temp_c10 = INT16_MIN;

  /* Start DMA circular reception */
  HAL_UART_Receive_DMA(&huart1, dmaRxBuf, DMA_RX_BUF_SIZE);

  /* Disable Error Interrupt Enable (EIE) — HAL_UART_Receive_DMA enables it,
     but on STM32F1 the only way to clear FE/NE/ORE flags is reading DR, which
     steals bytes from DMA and causes cascading framing errors. We poll SR for
     error counters in cansult_tick() instead. */
  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);

  /* Enable IDLE line interrupt (not enabled by HAL_UART_Receive_DMA) */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* --- Poll CAN RX for debug & mode commands --- */
static void pollCanRx(void) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
      continue;
    if (rxHeader.DLC < 1) continue;
    if (rxHeader.StdId == CANSULT_CAN_ID_DBG_CMD) {
      switch (rxData[0]) {
      case 0: debugStream = false; break;
      case 1: debugStream = true; break;
      case 2: debugStream = !debugStream; break;
      }
    } else if (rxHeader.StdId == CANSULT_CAN_ID_MODE_CMD) {
      switch (rxData[0]) {
      case 0: exitAdapterMode(); break;
      case 1: enterAdapterMode(); break;
      case 2:
        if (mode == CANSULT_MODE_STREAM) enterAdapterMode();
        else exitAdapterMode();
        break;
      }
    }
  }
}

void cansult_tick(void) {
  /* Healthy state after HAL_CAN_Start is LISTENING. Anything else (RESET
     after brownout, READY after aborted Start, ERROR) means we can't TX.
     Backoff: Stop/DeInit/Init/Start can each spend ~10 ms in INAK timeout
     on a wedged peripheral — without a gate, the main loop collapses from
     ~1 kHz to ~30 Hz and starves UART/parser/watchdog. */
  static uint32_t lastCanRecoverAttempt = 0;
  uint32_t now = HAL_GetTick();
  if (hcan.State != HAL_CAN_STATE_LISTENING &&
      now - lastCanRecoverAttempt > 500ul) {
    lastCanRecoverAttempt = now;
    recoverCan();
  }

  pollCanRx();

  /* Restart DMA if it stopped unexpectedly */
  if (huart1.RxState == HAL_UART_STATE_READY) {
    dmaRxReadPos = 0;
    HAL_UART_Receive_DMA(&huart1, dmaRxBuf, DMA_RX_BUF_SIZE);
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    cansult_diag.dma_restart_count++;
  }

  /* Poll UART error flags and clear them (SR+DR sequence).
     Main loop runs at ~1 kHz; race with DMA (reading DR when RXNE set) is rare.
     Worst case: occasional byte loss during error periods (already unreliable). */
  {
    uint32_t sr = huart1.Instance->SR;
    uint32_t err = sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE);
    if (err) {
      (void)huart1.Instance->DR;  /* SR already read above; DR read clears flags */
      if (err & USART_SR_ORE) cansult_diag.uart_ore_count++;
      if (err & USART_SR_FE) {
        if (cansult_diag.uart_fe_count == 0) {
          /* Read temp live; the 1Hz-cached value can be 0 (BSS) before
             first sendDiagFrame or up to a second stale later. */
          cansult_diag.first_fe_temp_c10 = readMcuTempC10();
        }
        cansult_diag.uart_fe_count++;
      }
      if (err & USART_SR_NE) cansult_diag.uart_ne_count++;
    }
  }

  time = HAL_GetTick();
  if (mode == CANSULT_MODE_STREAM) {
    processEcuBytes();  /* drain & update tempTime FIRST */
    route();            /* then check watchdog */
  }
  /* In ADAPTER mode parser/watchdog/CAN stream are paused; BT module owns
     the UART line, MCU just keeps CAN diag alive + polls for exit cmd. */
  sendDiagFrame();
}
