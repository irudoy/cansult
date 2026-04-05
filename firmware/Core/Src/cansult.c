#include "cansult.h"
#include "uart_rx_buf.h"
#include "consult_parser.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan;

/* --- DMA RX --- */
#define DMA_RX_BUF_SIZE 64
static uint8_t dmaRxBuf[DMA_RX_BUF_SIZE];
static uint32_t dmaRxReadPos = 0;
static volatile bool uartIdleFlag = false;

/* --- Ring buffer (between DMA and parser) --- */
static uart_rx_buf_t rxBuf;

/* --- Protocol parser --- */
static consult_parser_t parser;

/* --- State --- */
static uint8_t prevState = 0xFF;
static bool forceStopStream = false;
static uint8_t heartbeat = 0;
static uint32_t time;
static uint32_t tempTime;
static uint32_t lastFrameSentTime;

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

/* --- UART TX (short timeout) --- */
static HAL_StatusTypeDef writeEcu(uint8_t b) {
  return HAL_UART_Transmit(&huart1, &b, 1, 5);
}

/* --- Batch UART TX --- */
static HAL_StatusTypeDef writeEcuBuf(const uint8_t *buf, uint16_t len) {
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

/* --- Debug: log state change over CAN --- */
static void logStateChange(void) {
  if (parser.state != prevState) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {parser.state, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = 0x665;
    txHeader.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
  }
  prevState = parser.state;
}

/* --- Process all buffered ECU bytes through parser --- */
static void processEcuBytes(void) {
  uint8_t byte;
  while (readEcuByte(&byte)) {
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
      tempTime = time;  /* watchdog: got data */
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
}

/* --- Send CAN frames with current data --- */
static void sendCanFrames(void) {
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  uint8_t txData[8];

  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8;

  txHeader.StdId = CANSULT_CAN_ID_1;
  for (int i = 0; i < 8; i++) txData[i] = parser.data[i];
  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

  txHeader.StdId = CANSULT_CAN_ID_2;
  for (int i = 0; i < 7; i++) txData[i] = parser.data[8 + i];
  txData[7] = 0xFF;
  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

  txHeader.StdId = CANSULT_CAN_ID_3;
  txData[0] = parser.data[15];
  txData[1] = parser.data[16];
  txData[2] = 0xFF;
  txData[3] = 0xFF;
  txData[4] = 0xFF;
  txData[5] = 0xFF;
  txData[6] = parser.fault_codes[0];
  txData[7] = heartbeat;
  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

  lastFrameSentTime = time;
  heartbeat++;
}

/* --- State machine router --- */
static void route(void) {
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
    if (time - tempTime > 2000ul)
      consult_parser_set_state(&parser, CONSULT_STATE_STARTUP);
    break;
  case CONSULT_STATE_IDLE:
    if (!forceStopStream)
      requestStreaming();
    break;
  case CONSULT_STATE_STREAMING:
    if (time - tempTime > 500ul) {
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

void cansult_init(void) {
  HAL_CAN_Start(&hcan);
  uart_rx_buf_init(&rxBuf);
  consult_parser_init(&parser);

  /* Start DMA circular reception */
  HAL_UART_Receive_DMA(&huart1, dmaRxBuf, DMA_RX_BUF_SIZE);

  /* Enable IDLE line interrupt (not enabled by HAL_UART_Receive_DMA) */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void cansult_tick(void) {
  if (CANSULT_DEBUG)
    logStateChange();

  /* Restart DMA if it stopped unexpectedly */
  if (huart1.RxState == HAL_UART_STATE_READY) {
    dmaRxReadPos = 0;
    HAL_UART_Receive_DMA(&huart1, dmaRxBuf, DMA_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  }

  time = HAL_GetTick();
  route();
  processEcuBytes();
}
