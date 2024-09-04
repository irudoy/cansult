#include "cansult.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan;

uint8_t state = CANSULT_STATE_STARTUP;

uint8_t frameReadState = CANSULT_FRSTATE_STREAM;
uint8_t frameReadCount = 0;
bool needReadFrame = false;

bool forceStopStream = false;

uint8_t rxEcuByte;
uint8_t currEcuByte;
uint8_t prevEcuByte;

uint8_t data[CANSULT_MSG_BYTES_SIZE];
char ecuPartNo[11] = {'2', '3', '7', '1', '0', '-'};

uint8_t heartbeat = 0;

uint32_t time;
uint32_t tempTime;
uint32_t lastFrameSentTime;

uint8_t currentFaultCodesCount = 0;
uint8_t currentFaultCodes[CANSULT_FAULT_CODES_BUFFER_SIZE];

/**
 * Write byte to ECU
 * */
static HAL_StatusTypeDef writeEcu(uint8_t b) {
  return HAL_UART_Transmit(&huart1, &b, 1, 1000);
}

/**
 * Read byte from ECU
 * */
static HAL_StatusTypeDef readEcu() {
  return HAL_UART_Receive(&huart1, &rxEcuByte, 1, 1000);
}

/**
 * Start frame reader with state
 * */
static void startFrameReader(uint8_t frstate) {
  frameReadState = frstate;
  needReadFrame = true;
  frameReadCount = 0;
}

/**
 * Reset frame reader
 * */
static void resetFrameReader() {
  needReadFrame = false;
  frameReadCount = 0;
}

/**
 * Reset fault codes count
 * Fill buffer with 0xFF
 * */
static void resetFaultCodesReader() {
  currentFaultCodesCount = 0;
  for (uint8_t i = 0; i < CANSULT_FAULT_CODES_BUFFER_SIZE; i++) {
    currentFaultCodes[i] = 0xFF;
  }
}

/**
 * Send stop stream command
 * Change state to IDLE
 * Reset frame reader
 *
 * Empty serial stream
 * */
static void stopStream() {
  state = CANSULT_STATE_IDLE;
  resetFrameReader();
  writeEcu(CANSULT_ECU_COMMAND_STOP_STREAM);
  HAL_Delay(100);
  while (readEcu() == HAL_OK) {
    // Clear UART buffer
  }
}

/**
 * Send ECU init byte sequences
 * Doing it twice
 *
 * Switching state to CANSULT_STATE_INITIALIZING
 *
 * Also storing current time there, for future stuck checks
 * */
static void initECU() {
  stopStream();
  state = CANSULT_STATE_INITIALIZING;
  writeEcu(CANSULT_ECU_COMMAND_NULL);
  writeEcu(CANSULT_ECU_COMMAND_NULL);
  writeEcu(CANSULT_ECU_COMMAND_INIT);
  writeEcu(CANSULT_ECU_COMMAND_NULL);
  writeEcu(CANSULT_ECU_COMMAND_NULL);
  writeEcu(CANSULT_ECU_COMMAND_INIT);
  tempTime = time;
}

/**
 * Request some after-init data
 * Eg Ecu Part Number
 *
 * Switching state to CANSULT_STATE_WAITING_ECU_RESPONSE
 * */
static void postInit() {
  stopStream();
  state = CANSULT_STATE_WAITING_ECU_RESPONSE;
  writeEcu(CANSULT_ECU_COMMAND_ECU_INFO);
  writeEcu(CANSULT_ECU_COMMAND_TERM);
}

/**
 * Request ECU stream with all the data
 *
 * Switching state to CANSULT_STATE_STREAMING
 *
 * Size defined in CANSULT_MSG_BYTES_SIZE
 * */
static void requestStreaming() {
  stopStream();
  state = CANSULT_STATE_STREAMING;

  /**
   * This registers is valid for RB25DET NEO Y33 ECU (23710-XXXXX TODO)
   *
   * When ECU responses to CANSULT_ECU_COMMAND_READ_REGISTER it sends something like:
   * 0xA5 0x0C - It's OK, first byte is inverted 0x5A, second byte is requested register
   * 0xA5 0x08 - Same
   * ...
   * 0xA5 0xFE - Error there, second byte is 0xFE. It means that ECU doesn't
   * supports this and it must be removed
   * ...
   * */
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_BATTERY_VOLTAGE);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_COOLANT_TEMP);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_IGNITION_TIMING);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_LEFT_O2);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_TPS);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_AAC_VALVE);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_LEFT_AF_ALPHA);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_LEFT_AF_ALPHA_SELFLEARN);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_VEHICLE_SPEED);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_TACH_MSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_TACH_LSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_INJ_TIME_MSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_INJ_TIME_LSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_LEFT_MAF_MSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_LEFT_MAF_LSB);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_BIT_1);
  writeEcu(CANSULT_ECU_COMMAND_READ_REGISTER);
  writeEcu(CANSULT_ECU_REGISTER_BIT_2);
  writeEcu(CANSULT_ECU_COMMAND_TERM);
}

/**
 * Read DTC (fault codes)
 * */
static void readFaultCodes() {
  forceStopStream = true;
  stopStream();
  state = CANSULT_STATE_WAITING_ECU_RESPONSE;
  writeEcu(CANSULT_ECU_COMMAND_SELF_DIAG);
  writeEcu(CANSULT_ECU_COMMAND_TERM);
}

/**
 * Stop current stream and disable forceStopStream
 * */
static void resumeMainStream() {
  stopStream();
  forceStopStream = false;
}

/**
 * ECU responds with the same command byte but having inverted all bits as an error check
 * */
static bool errorCheckCommandByte(uint8_t commandByte, uint8_t errorCheckByte) {
  return commandByte == (uint8_t)
  ~errorCheckByte;
}

/**
 * Main ECU response processor
 * */
static void processECUInputByte() {
  if (readEcu() == HAL_OK) {
    prevEcuByte = currEcuByte;
    currEcuByte = rxEcuByte;

    // Reading ECU response byte-by-byte
    if (needReadFrame) {
      tempTime = time; // reading ECU, streaming OK

      switch (frameReadState) {
        // MAIN STREAM
        case CANSULT_FRSTATE_STREAM:
          data[frameReadCount] = currEcuByte;
          frameReadCount++;
          if (frameReadCount == CANSULT_MSG_BYTES_SIZE) {
            resetFrameReader();
          }
          break;

          // ECU PART NUMBER
        case CANSULT_FRSTATE_ECU_INFO:
          if (frameReadCount >= 19) {
            ecuPartNo[frameReadCount - 19 + 6] = (char) currEcuByte;
          }
          frameReadCount++;
          if (frameReadCount == 24) {
            resetFrameReader();
            readFaultCodes();
          }
          break;

          // FAULT CODES
        case CANSULT_FRSTATE_FAULT_CODES:
          if (prevEcuByte == CANSULT_ECU_REGISTER_NULL) {
            currentFaultCodesCount = currEcuByte;
          } else if (currEcuByte != CANSULT_ECU_REGISTER_NULL) {
            currentFaultCodes[frameReadCount] = currEcuByte;
            frameReadCount++;
            if (frameReadCount == currentFaultCodesCount) {
              resetFrameReader();
              resumeMainStream();
            }
          }
          break;

        default:
          break;

      }

      // ECU Init OK, moving forward
    } else if (state == CANSULT_STATE_INITIALIZING && errorCheckCommandByte(currEcuByte, CANSULT_ECU_COMMAND_INIT)) {

      state = CANSULT_STATE_POST_INIT;

      // Something was requested, checking ECU response
    } else if (state == CANSULT_STATE_WAITING_ECU_RESPONSE) {

      // Got ECU Part number
      if (errorCheckCommandByte(currEcuByte, CANSULT_ECU_COMMAND_ECU_INFO)) {
        startFrameReader(CANSULT_FRSTATE_ECU_INFO);
      }

      // Got Self Diag Codes
      if (errorCheckCommandByte(currEcuByte, CANSULT_ECU_COMMAND_SELF_DIAG)) {
        startFrameReader(CANSULT_FRSTATE_FAULT_CODES);
        resetFaultCodesReader();
      }

      // Got Clean DTC Codes Success Response
      if (errorCheckCommandByte(currEcuByte, CANSULT_ECU_COMMAND_CLEAR_CODES)) {
        readFaultCodes();
      }

      // Streaming main frames
    } else if (state == CANSULT_STATE_STREAMING) {

      // Unsupported register detected
      if (currEcuByte == 0xFE && prevEcuByte == (uint8_t)~CANSULT_ECU_COMMAND_READ_REGISTER) {
        // TODO: Handle Error
      }

      // Got right data, parsing
      if (currEcuByte == CANSULT_MSG_BYTES_SIZE && prevEcuByte == CANSULT_ECU_REGISTER_NULL) {
        startFrameReader(CANSULT_FRSTATE_STREAM);
      }

    }
  }
}

/**
 * Main router by state
 * */
static void route() {
  switch (state) {
    case CANSULT_STATE_STARTUP:
      initECU();
      break;
    case CANSULT_STATE_INITIALIZING:
      if (time - tempTime > 1000ul) {
        state = CANSULT_STATE_STARTUP;
      }
      break;
    case CANSULT_STATE_POST_INIT:
      postInit();
      break;
    case CANSULT_STATE_WAITING_ECU_RESPONSE:
      break;
    case CANSULT_STATE_IDLE:
      if (!forceStopStream) {
        requestStreaming();
      }
      break;
    case CANSULT_STATE_STREAMING:
      if (time - tempTime > 3000ul) {
        state = CANSULT_STATE_STARTUP;
      }
      if (time - lastFrameSentTime > CANSULT_DATA_RATE) {
        CAN_TxHeaderTypeDef txHeader;
        uint32_t txMailbox;
        uint8_t txData[8];

        txHeader.IDE = CAN_ID_STD;
        txHeader.RTR = CAN_RTR_DATA;

        txHeader.StdId = CANSULT_CAN_ID_1;
        txHeader.DLC = 8;
        txData[0] = data[0];
        txData[1] = data[1];
        txData[2] = data[2];
        txData[3] = data[3];
        txData[4] = data[4];
        txData[5] = data[5];
        txData[6] = data[6];
        txData[7] = data[7];
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

        txHeader.StdId = CANSULT_CAN_ID_2;
        txData[0] = data[8];
        txData[1] = data[9];
        txData[2] = data[10];
        txData[3] = data[11];
        txData[4] = data[12];
        txData[5] = data[13];
        txData[6] = data[14];
        txData[7] = 0xFF;
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

        txHeader.StdId = CANSULT_CAN_ID_3;
        txData[0] = data[15];
        txData[1] = data[16];
        txData[2] = 0xFF;
        txData[3] = 0xFF;
        txData[4] = 0xFF;
        txData[5] = 0xFF;
        txData[6] = currentFaultCodes[0];
        txData[7] = heartbeat;
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

        lastFrameSentTime = time;
        heartbeat = heartbeat == 255 ? 0 : heartbeat + 1;
      }
      break;
    default:
      break;
  }
}

void cansult_init() {
  HAL_CAN_Start(&hcan);
  resetFaultCodesReader();
}

void cansult_tick() {
  time = HAL_GetTick();
  route();
  processECUInputByte();
}
