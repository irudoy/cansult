//////////////////////////////////////////////
// Debug control
#define DEBUG_STATE true
#define DEBUG_SERIAL_PRINT true
#define DEBUG_INPUT_BYTES false
// Debug control
//////////////////////////////////////////////

//////////////////////////////////////////////
// Serial
#define Consult Serial3
#define DebugSerial Serial
// Serial
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU Commands
#define ECU_COMMAND_INIT 0xEF
#define ECU_COMMAND_NULL 0xFF
#define ECU_COMMAND_READ_REGISTER 0x5A
#define ECU_COMMAND_SELF_DIAG 0xD1
#define ECU_COMMAND_CLEAR_CODES 0xC1
#define ECU_COMMAND_ECU_INFO 0xD0
#define ECU_COMMAND_TERM 0xF0
#define ECU_COMMAND_STOP_STREAM 0x3
// ECU Commands
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU Registers single-byte
#define ECU_REGISTER_NULL 0xFF
#define ECU_REGISTER_COOLANT_TEMP 0x08
#define ECU_REGISTER_VEHICLE_SPEED 0x0B
#define ECU_REGISTER_BATTERY_VOLTAGE 0x0C
#define ECU_REGISTER_IGNITION_TIMING 0x16
#define ECU_REGISTER_LEFT_O2 0x09
#define ECU_REGISTER_TPS 0x0D
#define ECU_REGISTER_EGT 0x12
#define ECU_REGISTER_AAC_VALVE 0x17
#define ECU_REGISTER_LEFT_AF_ALPHA 0x1A
#define ECU_REGISTER_LEFT_AF_ALPHA_SELFLEARN 0x1C
// ECU Registers single-byte
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU Registers multi-byte
#define ECU_REGISTER_TACH_MSB 0x00
#define ECU_REGISTER_TACH_LSB 0x01
#define ECU_REGISTER_LEFT_MAF_MSB 0x04
#define ECU_REGISTER_LEFT_MAF_LSB 0x05
#define ECU_REGISTER_INJ_TIME_MSB 0x14
#define ECU_REGISTER_INJ_TIME_LSB 0x15
// ECU Registers multi-byte
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU digital (Bit) registers
#define ECU_REGISTER_BIT_1 0x13
#define ECU_REGISTER_BIT_2 0x1E
// ECU digital (Bit) registers
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU bitmasks for ECU_REGISTER_BIT_1
#define BITMASK_NEUTRAL_SW 0b00000100
#define BITMASK_START_SIGNAL 0b00000010
#define BITMASK_THROTTLE_CLOSED 0b00000001
//////////////////////////////////////////////

//////////////////////////////////////////////
// ECU bitmasks for ECU_REGISTER_BIT_2
#define BITMASK_FUEL_PUMP_RELAY 0b01000000
#define BITMASK_VTC_SOLENOID 0b00100000
//////////////////////////////////////////////

// STREAM FRAME SIZE
#define MSG_BYTES_SIZE 17

// STATE
#define STATE_STARTUP 0
#define STATE_INITIALIZING 1
#define STATE_POST_INIT 2
#define STATE_WAITING_ECU_RESPONSE 3
#define STATE_IDLE 4
#define STATE_STREAMING 5

uint8_t state = STATE_STARTUP;
uint8_t prevState;

// Frame read state
#define FRSTATE_STREAM 0
#define FRSTATE_ECU_INFO 1
#define FRSTATE_FAULT_CODES 2

uint8_t frameReadState = FRSTATE_STREAM;
uint8_t frameReadCount = 0;
bool needReadFrame = false;

bool forceStopStream = false;

byte ecuByte;
byte prevEcuByte;

byte data[MSG_BYTES_SIZE];
char ecuPartNo[11] = { '2', '3', '7', '1', '0', '-' };

unsigned long time;
unsigned long tempTime;

#define FAULT_CODES_BUFFER_SIZE 64
uint8_t currentFaultCodesCount = 0;
byte currentFaultCodes[FAULT_CODES_BUFFER_SIZE];

void setup() {
  DebugSerial.begin(115200);
  Consult.begin(9600);

  resetFaultCodesReader();
}

void loop() {
  time = millis();

  if (DEBUG_STATE) {
    logStateChange();
  }

  route();
  processECUInputByte();

  processDebugSerial();
}

/**
 * TMP: processing serial commands for debug
 * */
void processDebugSerial() {
  if (DebugSerial.available() > 0) {
    int command = DebugSerial.read();

    // TMP: Read coodes
    // 1
    if (command == 49) {
      DebugSerial.println("Reading err codes");
      readFaultCodes();
    }

    // TMP: Start the stream
    // 2
    if (command == 50) {
      DebugSerial.println("Streaming started");
      forceStopStream = false;
      requestStreaming();
    }

    // TMP: Force stop the stream
    // 3
    if (command == 51) {
      DebugSerial.println("Streaming stopped");
      forceStopStream = true;
      stopStream();
    }

    // TMP: Clear DTC
    // 4
    if (command == 52) {
      DebugSerial.println("Clearing err codes");
      forceStopStream = true;
      stopStream();
      state = STATE_WAITING_ECU_RESPONSE;
      writeEcu(ECU_COMMAND_CLEAR_CODES);
    }

    // TMP: Print all the data
    // 5
    if (command == 53) {
      int voltageMv = data[0] * 80;
      int cltDegC = data[1] - 50;
      int ignTimingDegBTDC = 110 - data[2];
      int O2VoltageMv = data[3] * 10;
      int TPSVoltageMv = data[4] * 20;
      int AACValvePerc = data[6] / 2;
      int AFAlphaPerc = data[7];
      int AFAlphaSLPerc = data[8];
      int vehicleSpeedKph = data[9] * 2;
      int rpm = ((data[10] << 8) + data[11]) * 12.5;
      float injTimeMs = ((data[12] << 8) + data[13]) / 100.0;
      int mafVoltageMv = ((data[14] << 8) + data[15]) * 5;

      bool neutralSwitch = data[16] & BITMASK_NEUTRAL_SW;
      bool startSignal = data[16] & BITMASK_START_SIGNAL;
      bool throttleClosed = data[16] & BITMASK_THROTTLE_CLOSED;
      bool fuelPumpRelay = data[17] & BITMASK_FUEL_PUMP_RELAY;
      bool vtcSolenoid = data[18] & BITMASK_VTC_SOLENOID;

      DebugSerial.print("Voltage: "); DebugSerial.print(voltageMv / 1000.0); DebugSerial.print("V; ");
      DebugSerial.print("CLT: "); DebugSerial.print(cltDegC); DebugSerial.print("C; ");
      DebugSerial.print("IGN TIMING: "); DebugSerial.print(ignTimingDegBTDC); DebugSerial.print("; ");
      DebugSerial.print("O2: "); DebugSerial.print(O2VoltageMv / 1000.0); DebugSerial.print("V; ");
      DebugSerial.print("TPS: "); DebugSerial.print(TPSVoltageMv / 1000.0); DebugSerial.print("V; ");
      DebugSerial.print("AAC: "); DebugSerial.print(AACValvePerc); DebugSerial.print("%; ");
      DebugSerial.print("A/F Alpha: "); DebugSerial.print(AFAlphaPerc); DebugSerial.print("%; ");
      DebugSerial.print("A/F Alpha (Self-Learn): "); DebugSerial.print(AFAlphaSLPerc); DebugSerial.print("%; ");
      DebugSerial.print("Speed: "); DebugSerial.print(vehicleSpeedKph); DebugSerial.print("km/h; ");
      DebugSerial.print("Tach: "); DebugSerial.print(rpm); DebugSerial.print("; ");
      DebugSerial.print("Inj Time: "); DebugSerial.print(injTimeMs); DebugSerial.print("ms; ");
      DebugSerial.print("MAF: "); DebugSerial.print(mafVoltageMv / 1000.0); DebugSerial.print("V; ");
      DebugSerial.println();
      DebugSerial.print("N: "); DebugSerial.print(neutralSwitch ? 1 : 0); DebugSerial.print("; ");
      DebugSerial.print("ST: "); DebugSerial.print(startSignal ? 1 : 0); DebugSerial.print("; ");
      DebugSerial.print("TH: "); DebugSerial.print(throttleClosed ? 1 : 0); DebugSerial.print("; ");
      DebugSerial.print("FP: "); DebugSerial.print(fuelPumpRelay ? 1 : 0); DebugSerial.print("; ");
      DebugSerial.print("VTC: "); DebugSerial.print(vtcSolenoid ? 1 : 0); DebugSerial.print("; ");
      DebugSerial.println();

      DebugSerial.print("ECU Part no: ");
      DebugSerial.println(ecuPartNo);

      if (currentFaultCodes[0] == 0xFF) {
        DebugSerial.println("No fault codes!");
      } else {
        DebugSerial.println("Fault codes:");
        for (uint8_t i = 0; i < FAULT_CODES_BUFFER_SIZE; i = i + 2) {
          if (currentFaultCodes[i] != 0xFF) {
            DebugSerial.print("Code: ");
            DebugSerial.print(currentFaultCodes[i], HEX);
            DebugSerial.print(" - ");
            printDTCDescriptionByCode(currentFaultCodes[i]);
            DebugSerial.print("; Starts: ");
            DebugSerial.println(currentFaultCodes[i + 1], DEC);
          }
        }
      }
    }
  }
}

/**
 * Main router by state
 * */
void route() {
  switch (state) {
    case STATE_STARTUP:
      if (DEBUG_SERIAL_PRINT) DebugSerial.println("Initializing...");
      initECU();
      break;
    case STATE_INITIALIZING:
      if (time - tempTime > 1000) {
        state = STATE_STARTUP;
      }
      break;
    case STATE_POST_INIT:
      if (DEBUG_SERIAL_PRINT) DebugSerial.println("Requesting ECU part number");
      postInit();
      break;
    case STATE_WAITING_ECU_RESPONSE:
      break;
    case STATE_IDLE:
      if (!forceStopStream) {
        if (DEBUG_SERIAL_PRINT) DebugSerial.println("Requesting stream...");
        requestStreaming();
      }
      break;
    case STATE_STREAMING:
      break;
    default:
      break;
  }
}

/**
 * Main ECU response processor
 * */
void processECUInputByte() {
  if (Consult.available() > 0) {
    prevEcuByte = ecuByte;
    ecuByte = readEcu();

    if (DEBUG_INPUT_BYTES) {
      DebugSerial.print(ecuByte, HEX);
      DebugSerial.print(" ");
      if (prevEcuByte == ECU_REGISTER_NULL && ecuByte == MSG_BYTES_SIZE) {
        DebugSerial.println();
      } else {
        DebugSerial.print(" ");
      }
    }

    // Reading ECU response byte-by-byte
    if (needReadFrame) {

      switch (frameReadState) {

        // MAIN STREAM
        case FRSTATE_STREAM:
          data[frameReadCount] = ecuByte;
          frameReadCount++;
          if (frameReadCount == MSG_BYTES_SIZE) {
            resetFrameReader();
          }
          break;

        // ECU PART NUMBER
        case FRSTATE_ECU_INFO:
          if (frameReadCount >= 19) {
            ecuPartNo[frameReadCount - 19 + 6] = (char)ecuByte;
          }
          frameReadCount++;
          if (frameReadCount == 24) {
            resetFrameReader();
            if (DEBUG_SERIAL_PRINT) DebugSerial.println("Got ECU Part Number");
            readFaultCodes();
          }
          break;

        // FAULT CODES
        case FRSTATE_FAULT_CODES:
          if (prevEcuByte == ECU_REGISTER_NULL) {
            currentFaultCodesCount = ecuByte;
          } else if (ecuByte != ECU_REGISTER_NULL) {
            currentFaultCodes[frameReadCount] = ecuByte;
            frameReadCount++;
            if (frameReadCount == currentFaultCodesCount) {
              if (DEBUG_SERIAL_PRINT) {
                DebugSerial.print("Got ");
                DebugSerial.print(currentFaultCodesCount / 2);
                DebugSerial.println(" fault codes");
              }
              resetFrameReader();
              resumeMainStream();
            }
          }
          break;

        default:
          break;

      }

    // ECU Init OK, moving forward
    } else if (state == STATE_INITIALIZING && errorCheckCommandByte(ecuByte, ECU_COMMAND_INIT)) {

      state = STATE_POST_INIT;
      if (DEBUG_SERIAL_PRINT) DebugSerial.println("Initialized succesfully!");

    // Something was requested, checking ECU response
    } else if (state == STATE_WAITING_ECU_RESPONSE) {

      // Got ECU Part number
      if (errorCheckCommandByte(ecuByte, ECU_COMMAND_ECU_INFO)) {
        if (DEBUG_SERIAL_PRINT) DebugSerial.println("Reading part number");
        startFrameReader(FRSTATE_ECU_INFO);
      }

      // Got Self Diag Codes
      if (errorCheckCommandByte(ecuByte, ECU_COMMAND_SELF_DIAG)) {
        if (DEBUG_SERIAL_PRINT) DebugSerial.println("Reading fault codes");
        startFrameReader(FRSTATE_FAULT_CODES);
        resetFaultCodesReader();
      }

      // Got Clean DTC Codes Success Response
      if (errorCheckCommandByte(ecuByte, ECU_COMMAND_CLEAR_CODES)) {
        if (DEBUG_SERIAL_PRINT) DebugSerial.println("Fault codes cleared succesfully");
        readFaultCodes();
      }

    // Streaming main frames
    } else if (state == STATE_STREAMING) {

      // Unsupported register detected
      if (ecuByte == 0xFE && prevEcuByte == (byte)~ECU_COMMAND_READ_REGISTER) {
        if (DEBUG_SERIAL_PRINT) DebugSerial.println("ERROR: Unsupported register");
      }

      // Got right data, parsing
      if (ecuByte == MSG_BYTES_SIZE && prevEcuByte == ECU_REGISTER_NULL) {
        startFrameReader(FRSTATE_STREAM);
      }

    }
  }
}

/**
 * Debug state change logger
 * */
void logStateChange() {
  if (state != prevState) {
    DebugSerial.print("State changed: ");
    switch (state) {
      case STATE_STARTUP: DebugSerial.println("STATE_STARTUP"); break;
      case STATE_INITIALIZING: DebugSerial.println("STATE_INITIALIZING"); break;
      case STATE_POST_INIT: DebugSerial.println("STATE_POST_INIT"); break;
      case STATE_WAITING_ECU_RESPONSE: DebugSerial.println("STATE_WAITING_ECU_RESPONSE"); break;
      case STATE_IDLE: DebugSerial.println("STATE_IDLE"); break;
      case STATE_STREAMING: DebugSerial.println("STATE_STREAMING"); break;
      default: DebugSerial.println(state); break;
    }
  }
  prevState = state;
}

/**
 * Send ECU init byte sequences
 * Doing it twice
 * 
 * Switching state to STATE_INITIALIZING
 * 
 * Also storing current time there, for future stuck checks
 * */
void initECU() {
  stopStream();
  state = STATE_INITIALIZING;
  writeEcu(ECU_COMMAND_NULL);
  writeEcu(ECU_COMMAND_NULL);
  writeEcu(ECU_COMMAND_INIT);
  writeEcu(ECU_COMMAND_NULL);
  writeEcu(ECU_COMMAND_NULL);
  writeEcu(ECU_COMMAND_INIT);
  tempTime = time;
}

/**
 * Request some after-init data
 * Eg Ecu Part Number
 * 
 * Switching state to STATE_WAITING_ECU_RESPONSE
 * */
void postInit() {
  stopStream();
  state = STATE_WAITING_ECU_RESPONSE;
  writeEcu(ECU_COMMAND_ECU_INFO);
  writeEcu(ECU_COMMAND_TERM);
}

/**
 * Request ECU stream with all the data
 * 
 * Switching state to STATE_STREAMING
 * 
 * Size defined in MSG_BYTES_SIZE
 * */
void requestStreaming() {
  stopStream();
  state = STATE_STREAMING;

  /**
   * This registers is valid for RB25DET NEO Y33 ECU (23710-XXXXX TODO)
   * 
   * When ECU responses to ECU_COMMAND_READ_REGISTER it sends something like:
   * 0xA5 0x0C - It's OK, first byte is inverted 0x5A, second byte is requested register
   * 0xA5 0x08 - Same
   * ...
   * 0xA5 0xFE - Error there, second byte is 0xFE. It means that ECU doesn't supports this and it must be removed
   * ...
   * */
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_BATTERY_VOLTAGE);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_COOLANT_TEMP);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_IGNITION_TIMING);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_LEFT_O2);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_TPS);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_AAC_VALVE);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_LEFT_AF_ALPHA);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_LEFT_AF_ALPHA_SELFLEARN);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_VEHICLE_SPEED);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_TACH_MSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_TACH_LSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_INJ_TIME_MSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_INJ_TIME_LSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_LEFT_MAF_MSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_LEFT_MAF_LSB);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_BIT_1);
  writeEcu(ECU_COMMAND_READ_REGISTER);
  writeEcu(ECU_REGISTER_BIT_2);
  writeEcu(ECU_COMMAND_TERM);
}

/**
 * Read DTC (fault codes)
 * */
void readFaultCodes() {
  forceStopStream = true;
  stopStream();
  state = STATE_WAITING_ECU_RESPONSE;
  writeEcu(ECU_COMMAND_SELF_DIAG);
  writeEcu(ECU_COMMAND_TERM);
}

/**
 * Reset fault codes count
 * Fill buffer with 0xFF
 * */
void resetFaultCodesReader() {
  currentFaultCodesCount = 0;
  for (uint8_t i = 0; i < FAULT_CODES_BUFFER_SIZE; i++) {
    currentFaultCodes[i] = 0xFF;
  }
}

/**
 * Start frame reader with state
 * */
void startFrameReader(uint8_t frstate) {
  frameReadState = frstate;
  needReadFrame = true;
  frameReadCount = 0;
}

/**
 * Reset frame reader
 * */
void resetFrameReader() {
  needReadFrame = false;
  frameReadCount = 0;
}

/**
 * Stop current stream and disable forceStopStream
 * */
void resumeMainStream() {
  stopStream();
  forceStopStream = false;
}

/**
 * Write byte to ECU
 * */
void writeEcu(byte b) {
  Consult.write((byte)b);
}

/**
 * Read byte from ECU
 * */
byte readEcu() {
  return (byte)Consult.read();
}

/**
 * ECU responds with the same command byte but having inverted all bits as an error check
 * */
boolean errorCheckCommandByte(byte commandByte, byte errorCheckByte) {
  return commandByte == (byte)~errorCheckByte;
}

/**
 * Send stop stream command
 * Change state to IDLE
 * Reset frame reader
 * 
 * Empty serial stream
 * */
void stopStream() {
  state = STATE_IDLE;
  resetFrameReader();
  writeEcu(ECU_COMMAND_STOP_STREAM);
  delay(100);
  Consult.flush();
  while (Consult.available() > 0) {
    Consult.read();
  }
}

void printDTCDescriptionByCode(byte code) {
  switch (code) {
    case 0x11: DebugSerial.print("CRANKSHAFT POSITION SENSOR"); break;
    case 0x12: DebugSerial.print("MASS AIRFLOW SENSOR"); break;
    case 0x13: DebugSerial.print("ENGINE COOLANT TEMPERATURE SENSOR"); break;
    case 0x14: DebugSerial.print("VEHICLE SPEED SENSOR"); break;
    case 0x21: DebugSerial.print("IGNITION SIGNAL"); break;
    case 0x31: DebugSerial.print("ECM"); break;
    case 0x32: DebugSerial.print("EGR FUNCTION"); break;
    case 0x33: DebugSerial.print("OXYGEN SENSOR"); break;
    case 0x34: DebugSerial.print("KNOCK SENSOR"); break;
    case 0x35: DebugSerial.print("EGR TEMPERATURE SENSOR"); break;
    case 0x42: DebugSerial.print("FUEL TEMPERATURE SENSOR"); break;
    case 0x43: DebugSerial.print("THROTTLE POSITION SENSOR"); break;
    case 0x45: DebugSerial.print("INJECTOR LEAK"); break;
    case 0x51: DebugSerial.print("INJECTOR CIRCUIT"); break;
    case 0x53: DebugSerial.print("OXYGEN SENSOR"); break;
    case 0x55: DebugSerial.print("NO MALFUNCTION"); break;
    case 0x54: DebugSerial.print("AUTO TRANSMISSION SIGNAL"); break;
    default: DebugSerial.print("UNKNOWN"); break;
  }
}