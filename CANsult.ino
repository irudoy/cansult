#define ECU_COMMAND_READ_REGISTER 0x5A
#define ECU_COMMAND_SELF_DIAG 0xD1
#define ECU_COMMAND_CLEAR_CODES 0xC1
#define ECU_COMMAND_ECU_INFO 0xD0
#define ECU_COMMAND_TERM 0xF0
#define ECU_COMMAND_STOP_STREAM 0x3

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

#define ECU_REGISTER_TACH_MSB 0x00
#define ECU_REGISTER_TACH_LSB 0x01
#define ECU_REGISTER_LEFT_MAF_MSB 0x04
#define ECU_REGISTER_LEFT_MAF_LSB 0x05
#define ECU_REGISTER_INJ_TIME_MSB 0x14
#define ECU_REGISTER_INJ_TIME_LSB 0x15

#define ECU_REGISTER_BIT_1 0x13
#define ECU_REGISTER_BIT_2 0x1E

// ECU_REGISTER_BIT_1
#define BITMASK_NEUTRAL_SW 0b00000100
#define BITMASK_START_SIGNAL 0b00000010
#define BITMASK_THROTTLE_CLOSED 0b00000001

// ECU_REGISTER_BIT_2
#define BITMASK_FUEL_PUMP_RELAY 0b01000000
#define BITMASK_VTC_SOLENOID 0b00100000

#define ECU_REGISTER_NULL 0xFF

#define ECU_MSG_INIT_OK 0x10

// STREAM FRAME SIZE
#define MSG_BYTES_SIZE 18

#define STATE_STARTUP 0
#define STATE_INITIALIZING 1
#define STATE_IDLE 2
#define STATE_STREAMING 3

uint8_t state = STATE_STARTUP;

bool forceStopStream = false;

byte ecuByte;
byte prevEcuByte;

byte data[MSG_BYTES_SIZE];
char ecuPartNo[12];

int streamReadCount = 0;
bool needReadStream = false;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
}

void loop() {
  switch (state) {
    case STATE_STARTUP:
      Serial.println("Initializing...");
      initECU();
      break;
    case STATE_INITIALIZING:
      break;
    case STATE_IDLE:
      if (!forceStopStream) {
        Serial.println("Requesting stream...");
        requestStreaming();
      }
      break;
    case STATE_STREAMING:
      break;
    default:
      break;
  }

  if (Serial3.available() > 0) {
    prevEcuByte = ecuByte;
    ecuByte = readEcu();

    // DEBUG
    // Serial.print(ecuByte, HEX);
    // Serial.print(" ");
    // if (prevEcuByte == ECU_REGISTER_NULL && ecuByte == MSG_BYTES_SIZE) {
    //   Serial.println();
    // } else {
    //   Serial.print(" ");
    // }
    // DEBUG

    if (needReadStream) {
      data[streamReadCount] = ecuByte;
      streamReadCount++;
      if (streamReadCount == MSG_BYTES_SIZE) {
        needReadStream = false;
        streamReadCount = 0;
      }
    } else if (state == STATE_INITIALIZING && ecuByte == ECU_MSG_INIT_OK) { // init ok
      state = STATE_IDLE;
      Serial.println("Initialized succesfully!");
    } else if (ecuByte == 0x2F) { // read ecu part no; TODO: shouldn't work anymore
      ecuPartNo[0] = '2';
      ecuPartNo[1] = '3';
      ecuPartNo[2] = '7';
      ecuPartNo[3] = '1';
      ecuPartNo[4] = '0';
      ecuPartNo[5] = '-';

      readEcu();

      int curPos = 6;

      for (int x = 1; x <= 23; x++) {
        ecuByte = readEcu();
        if (x >= 19 && x <= 23) {
          ecuPartNo[curPos] = (char)ecuByte;
          curPos++;
        }
      }

      stopStream();
    } else if (ecuByte == MSG_BYTES_SIZE && prevEcuByte == ECU_REGISTER_NULL) { // read stream
      needReadStream = true;
    }
  }

  if (Serial.available() > 0) {
    int command = Serial.read();
    if (command == 50) { // START STREAM 2
      Serial.println("Streaming started");
      forceStopStream = false;
      requestStreaming();
    }
    if (command == 51) { // FORCE STOP STREAM 3
      Serial.println("Streaming stopped");
      forceStopStream = true;
      stopStream();
    }
    if (command == 52) { // ecu part number 4
      Serial.println("Requesting ECU part number");
      stopStream();
      writeEcu(ECU_COMMAND_ECU_INFO);
      writeEcu(ECU_COMMAND_TERM);
    }
    if (command == 53) { // print data 5
      // for (int i = 0; i <= 8; i++) {
      //   Serial.print("0x");
      //   Serial.print(data[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();

      // 0 ECU_REGISTER_BATTERY_VOLTAGE
      // 1 ECU_REGISTER_COOLANT_TEMP // Value-50 (deg C)
      // 2 ECU_REGISTER_IGNITION_TIMING // 110 – Value (Deg BTDC)
      // 3 ECU_REGISTER_LEFT_O2 //  Value * 10 (mV)
      // 4 ECU_REGISTER_TPS // Value * 20 (mV)
      // 5 ECU_REGISTER_EGT // Value * 20 (mV)
      // 6 ECU_REGISTER_AAC_VALVE // Value / 2 (%)
      // 7 ECU_REGISTER_LEFT_AF_ALPHA // Value (%)
      // 8 ECU_REGISTER_LEFT_AF_ALPHA_SELFLEARN // Value (%)
      // 9 ECU_REGISTER_VEHICLE_SPEED // Value * 2 (kph)
      // 10 ECU_REGISTER_TACH_MSB
      // 11 ECU_REGISTER_TACH_LSB // Value * 12.5 (RPM)
      // 12 ECU_REGISTER_INJ_TIME_MSB
      // 13 ECU_REGISTER_INJ_TIME_LSB // Value / 100 (mS)
      // 14 ECU_REGISTER_LEFT_MAF_MSB
      // 15 ECU_REGISTER_LEFT_MAF_LSB // Value * 5 (mV)
      // 16 ECU_REGISTER_BIT_1
      // 17 ECU_REGISTER_BIT_2

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

      // Serial.print("ECU Part no: ");
      // Serial.println(ecuPartNo);
    }
  }
}

void initECU() {
  stopStream();
  state = STATE_INITIALIZING;
  writeEcu(0xFF);
  writeEcu(0xFF);
  writeEcu(0xEF);
  writeEcu(0xFF);
  writeEcu(0xFF);
  writeEcu(0xEF);
}

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

void writeEcu(byte b) {
  Serial3.write((byte)b);
}

byte readEcu() {
  return (byte)Serial3.read();
}

void stopStream() {
  state = STATE_IDLE;

  needReadStream = false;
  streamReadCount = 0;

  writeEcu(ECU_COMMAND_STOP_STREAM);
  delay(100);
  Serial3.flush();
  while (Serial3.available() > 0) {
    Serial3.read();
  }
}
