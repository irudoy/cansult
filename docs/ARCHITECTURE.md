# Architecture

## Data Flow

```
ECU (9600 8N1)
  │
  ▼
USART1 RX (PA10)
  │
  ▼ DMA1 Channel 5, Circular, 64-byte buffer
  │
  ▼ drainDmaToRingBuf() — polls DMA NDTR counter
  │
  ▼ uart_rx_buf (128-byte ring buffer)
  │
  ▼ consult_parser_feed() — byte-by-byte state machine
  │
  ├─► CONSULT_EVENT_STREAM_FRAME → parser.data[17] updated
  ├─► CONSULT_EVENT_INIT_OK → state → POST_INIT
  ├─► CONSULT_EVENT_ECU_INFO_READY → readFaultCodes()
  ├─► CONSULT_EVENT_FAULT_CODES_READY → resumeMainStream()
  └─► CONSULT_EVENT_CLEAR_CODES_OK → readFaultCodes()
  │
  ▼ sendCanFrames() @ 20Hz
  │
CAN TX → 0x666, 0x667, 0x668
```

## State Machine

```
STARTUP ──initECU()──► INITIALIZING ──0x10 response──► POST_INIT
   ▲                       │ 3s timeout                    │
   └───────────────────────┘                          postInit()
                                                          │
                                                          ▼
                    IDLE ◄──resumeMainStream()── WAITING_ECU_RESPONSE
                     │                                │ 2s timeout
              requestStreaming()                       ▼
                     │                             STARTUP
                     ▼
                 STREAMING ──500ms no data──► STARTUP
                     │
              sendCanFrames() @ 20Hz
```

## Modules

### Lib/consult_parser (host-testable)

Nissan Consult protocol byte parser. Pure state machine, no HAL.

- `consult_parser_init()` — reset to STARTUP
- `consult_parser_feed(byte)` — process one ECU byte, returns event enum
- `consult_parser_set_state()` — HAL layer controls state transitions
- Handles: init response, stream frame sync (0xFF + 0x11), ECU info (24 bytes), fault codes (variable length, bounds-checked)

### Lib/uart_rx_buf (host-testable)

128-byte power-of-2 ring buffer. Single-producer/single-consumer from main loop.

- `push/pop/flush/available`
- Drops on full (no blocking)

### Core/Src/cansult.c (HAL glue)

- DMA circular RX → ring buffer drain (polls `__HAL_DMA_GET_COUNTER`)
- Non-blocking `readEcuByte()`: drain + pop
- Blocking TX with 5ms timeout (safe during DMA RX — separate HAL gState/RxState)
- `stopStream()`: sends 0x30 ×3, waits 200ms for 0xCF ACK, flushes ring buffer
- `requestStreaming()`: batch TX 35 bytes in single HAL call
- DMA auto-restart: if `huart1.RxState == READY`, re-init DMA

### Core/Src/stm32f1xx_it.c (ISR)

- `DMA1_Channel5_IRQHandler` — HAL DMA handler (CubeMX generated)
- `USART1_IRQHandler` — IDLE line detection (sets flag for main loop), clears ORE/FE/NE error flags

## UART/DMA Configuration

Configured via CubeMX (`cansult.ioc`):

| Parameter | Value |
|-----------|-------|
| USART1 Baud | 9600 |
| DMA1 Channel | 5 (USART1_RX) |
| DMA Mode | Circular |
| DMA Priority | High |
| DMA Buffer | 64 bytes |
| Ring Buffer | 128 bytes |
| NVIC DMA1_Ch5 | Priority 1 |
| NVIC USART1 | Priority 2 |
| Stack | 0x800 (2KB) |

## CAN Protocol

See `README.md` for frame layout and data conversion.

- CAN IDs: 0x666, 0x667, 0x668 (data), 0x665 (debug state)
- Bitrate: 500 kbit/s
- TX rate: 20Hz (50ms interval)
- 17 ECU registers streamed per cycle

## Consult Protocol

See `docs/UART_RELIABILITY.md` for detailed protocol analysis.

- Init: `0xFF 0xFF 0xEF` (sent twice), ECU responds `0x10`
- Stream request: `0x5A <reg> ... 0xF0`, ECU streams `0xFF <count=17> <data...>` continuously
- Stop: `0x30`, ECU responds `0xCF`
- Baud: 9600 8N1

## Testing

21 host tests via Unity (`make test`):

- `test_uart_rx_buf` (7 tests): push/pop, FIFO order, wraparound, full drops, flush
- `test_consult_parser` (14 tests): init response, stream frames, ECU info parsing, fault codes (incl. bounds check, zero count), state guards
