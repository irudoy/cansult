# Nissan Consult (14pin) to CAN adapter

This project aims to translate the Nissan Consult protocol to CAN bus. It is based on the STM32F103 microcontroller and custom-made board.
The adapter is both powered and communicates with the ECU through the 14-pin Consult connector.

## CAN bus

### Baud rate
500 Kbit/s

### Frames payload
|  ID   | DLC |     Byte 0      |    Byte 1    |     Byte 2      |    Byte 3    |    Byte 4    |    Byte 5    |     Byte 6     |         Byte 7          |
|-------|-----|-----------------|--------------|-----------------|--------------|--------------|--------------|----------------|-------------------------|
| 0x666 |   8 | BATTERY_VOLTAGE | COOLANT_TEMP | IGNITION_TIMING | LEFT_O2      | TPS          | AAC_VALVE    | LEFT_AF_ALPHA  | LEFT_AF_ALPHA_SELFLEARN |
| 0x667 |   8 | VEHICLE_SPEED   | TACH_MSB     | TACH_LSB        | INJ_TIME_MSB | INJ_TIME_LSB | LEFT_MAF_MSB | LEFT_MAF_LSB   |                         |
| 0x668 |   8 | BIT_1           | BIT_2        | DEVICE_VOLTAGE  |              |              |              | First DTC Code | Heartbeat               |

## Hardware

* STM32F103TBU6
* MCP2562
* MC74HC4060

<img src="hardware/export/pcb-f.png" />
<img src="hardware/export/pcb-b.png" />
