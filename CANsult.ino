//////////////////////////////////////////////
// Debug control
#define DEBUG_STATE true
#define DEBUG_SERIAL_PRINT true
#define DEBUG_INPUT_BYTES false
// Debug control
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
#define MSG_BYTES_SIZE 18

// STATE
#define STATE_STARTUP 0
#define STATE_INITIALIZING 1
#define STATE_POST_INIT_START 2
#define STATE_POST_INIT 3
#define STATE_IDLE 4
#define STATE_STREAMING 5

uint8_t state = STATE_STARTUP;
uint8_t prevState;

// Frame read state
#define FRSTATE_STREAM 0
#define FRSTATE_ECU_INFO 1

uint8_t frameReadState = FRSTATE_STREAM;
uint8_t frameReadCount = 0;
bool needReadFrame = false;

bool forceStopStream = false;

byte ecuByte;
byte prevEcuByte;

byte data[MSG_BYTES_SIZE];
char ecuPartNo[12];

unsigned long time;
unsigned long tempTime;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
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
  if (Serial.available() > 0) {
    int command = Serial.read();

    // TMP: Start the stream
    // 3
    if (command == 50) {
      Serial.println("Streaming started");
      forceStopStream = false;
      requestStreaming();
    }

    // TMP: Force stop the stream
    // 4
    if (command == 51) {
      Serial.println("Streaming stopped");
      forceStopStream = true;
      stopStream();
    }

    // TMP: Print all the data
    // 5
    if (command == 53) {
      int voltageMv = data[0] * 80;
      int cltDegC = data[1] - 50;
      int ignTimingDegBTDC = 110 - data[2];
      int O2VoltageMv = data[3] * 10;
      int TPSVoltageMv = data[4] * 20;
      int EGTVoltageMv = data[5] * 20;
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

      Serial.print("Voltage: "); Serial.print(voltageMv / 1000.0); Serial.print("V; ");
      Serial.print("CLT: "); Serial.print(cltDegC); Serial.print("C; ");
      Serial.print("IGN TIMING: "); Serial.print(ignTimingDegBTDC); Serial.print("; ");
      Serial.print("O2: "); Serial.print(O2VoltageMv / 1000.0); Serial.print("V; ");
      Serial.print("TPS: "); Serial.print(TPSVoltageMv / 1000.0); Serial.print("V; ");
      Serial.print("EGT: "); Serial.print(EGTVoltageMv / 1000.0); Serial.print("V; ");
      Serial.print("AAC: "); Serial.print(AACValvePerc); Serial.print("%; ");
      Serial.print("A/F Alpha: "); Serial.print(AFAlphaPerc); Serial.print("%; ");
      Serial.print("A/F Alpha (Self-Learn): "); Serial.print(AFAlphaSLPerc); Serial.print("%; ");
      Serial.print("Speed: "); Serial.print(vehicleSpeedKph); Serial.print("km/h; ");
      Serial.print("Tach: "); Serial.print(rpm); Serial.print("; ");
      Serial.print("Inj Time: "); Serial.print(injTimeMs); Serial.print("ms; ");
      Serial.print("MAF: "); Serial.print(mafVoltageMv / 1000.0); Serial.print("V; ");
      Serial.println();
      Serial.print("N: "); Serial.print(neutralSwitch ? 1 : 0); Serial.print("; ");
      Serial.print("ST: "); Serial.print(startSignal ? 1 : 0); Serial.print("; ");
      Serial.print("TH: "); Serial.print(throttleClosed ? 1 : 0); Serial.print("; ");
      Serial.print("FP: "); Serial.print(fuelPumpRelay ? 1 : 0); Serial.print("; ");
      Serial.print("VTC: "); Serial.print(vtcSolenoid ? 1 : 0); Serial.print("; ");
      Serial.println();

      Serial.print("ECU Part no: ");
      Serial.println(ecuPartNo);
    }
  }
}

/**
 * Main router by state
 * */
void route() {
  switch (state) {
    case STATE_STARTUP:
      if (DEBUG_SERIAL_PRINT) Serial.println("Initializing...");
      initECU();
      break;
    case STATE_INITIALIZING:
      break;
    case STATE_POST_INIT_START:
      if (DEBUG_SERIAL_PRINT) Serial.println("Requesting ECU part number");
      postInit();
      break;
    case STATE_POST_INIT:
      if (time - tempTime > 1000) {
        state = STATE_STARTUP;
      }
      break;
    case STATE_IDLE:
      if (!forceStopStream) {
        if (DEBUG_SERIAL_PRINT) Serial.println("Requesting stream...");
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
  if (Serial3.available() > 0) {
    prevEcuByte = ecuByte;
    ecuByte = readEcu();

    if (DEBUG_INPUT_BYTES) {
      Serial.print(ecuByte, HEX);
      Serial.print(" ");
      if (prevEcuByte == ECU_REGISTER_NULL && ecuByte == MSG_BYTES_SIZE) {
        Serial.println();
      } else {
        Serial.print(" ");
      }
    }

    if (needReadFrame) {
      switch (frameReadState) {
        case FRSTATE_STREAM:
          data[frameReadCount] = ecuByte;
          frameReadCount++;
          if (frameReadCount == MSG_BYTES_SIZE) {
            needReadFrame = false;
            frameReadCount = 0;
          }
          break;
        case FRSTATE_ECU_INFO:
          if (frameReadCount >= 18) {
            ecuPartNo[frameReadCount - 18 + 6] = (char)ecuByte;
          }
          frameReadCount++;
          if (frameReadCount == 23) {
            needReadFrame = false;
            frameReadCount = 0;
            if (DEBUG_SERIAL_PRINT) Serial.println("Got ECU Part Number");
            stopStream();
          }
          break;
        default:
          break;
      }
    } else if (state == STATE_INITIALIZING && errorCheckCommandByte(ecuByte, ECU_COMMAND_INIT)) {
      state = STATE_POST_INIT_START;
      if (DEBUG_SERIAL_PRINT) Serial.println("Initialized succesfully!");
    } else if (state == STATE_POST_INIT && errorCheckCommandByte(ecuByte, ECU_COMMAND_ECU_INFO)) {
      if (DEBUG_SERIAL_PRINT) Serial.println("Reading part number");
      needReadFrame = true;
      frameReadState = FRSTATE_ECU_INFO;
      ecuPartNo[0] = '2';
      ecuPartNo[1] = '3';
      ecuPartNo[2] = '7';
      ecuPartNo[3] = '1';
      ecuPartNo[4] = '0';
      ecuPartNo[5] = '-';
    } else if (state == STATE_STREAMING && ecuByte == MSG_BYTES_SIZE && prevEcuByte == ECU_REGISTER_NULL) {
      needReadFrame = true;
      frameReadState = FRSTATE_STREAM;
    }
  }
}

/**
 * Debug state change logger
 * */
void logStateChange() {
  if (state != prevState) {
    Serial.print("State changed: ");
    switch (state) {
      case STATE_STARTUP: Serial.println("STATE_STARTUP"); break;
      case STATE_INITIALIZING: Serial.println("STATE_INITIALIZING"); break;
      case STATE_POST_INIT_START: Serial.println("STATE_POST_INIT_START"); break;
      case STATE_POST_INIT: Serial.println("STATE_POST_INIT"); break;
      case STATE_IDLE: Serial.println("STATE_IDLE"); break;
      case STATE_STREAMING: Serial.println("STATE_STREAMING"); break;
      default: Serial.println(state); break;
    }
  }
  prevState = state;
}

/**
 * Send ECU init byte sequences
 * Doing it twice
 * 
 * Switching state to STATE_INITIALIZING
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
}

/**
 * Request some after-init data
 * Eg Ecu Part Number
 * 
 * Switching state to STATE_POST_INIT
 * 
 * Also storing cuurent time there, for future stuck checks
 * */
void postInit() {
  stopStream();
  state = STATE_POST_INIT;
  writeEcu(ECU_COMMAND_ECU_INFO);
  writeEcu(ECU_COMMAND_TERM);
  tempTime = time;
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
  writeEcu(ECU_REGISTER_EGT);
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
 * Write byte to ECU
 * */
void writeEcu(byte b) {
  Serial3.write((byte)b);
}

/**
 * Read byte from ECU
 * */
byte readEcu() {
  return (byte)Serial3.read();
}

/**
 * ECU responds with the same command byte but having inverted all bits as an error check
 * */
boolean errorCheckCommandByte(byte commandByte, byte errorCheckByte) {
  return commandByte != (byte)~errorCheckByte;
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

  needReadFrame = false;
  frameReadCount = 0;

  writeEcu(ECU_COMMAND_STOP_STREAM);
  delay(100);
  Serial3.flush();
  while (Serial3.available() > 0) {
    Serial3.read();
  }
}
