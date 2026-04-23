# Architecture

## Data Flow

```
ECU (9600 8N1)
  │
  ▼
USART1 RX (PA10)
  │
  ▼ DMA1 Channel 5, Circular, 128-byte buffer
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
CAN TX → 0x666, 0x667, 0x668 (stream data @ 20Hz)
         0x665 (diagnostics — state, rates, freshness @ 1Hz)
         0x66B (diagnostics — u16 error counters, MCU temp, mode @ 1Hz)
         0x669, 0x66A (debug UART stream, when enabled)
  │
CAN RX ← 0x66F (debug command: enable/disable UART stream)
         0x66D (mode command: STREAM ↔ ADAPTER / BT pass-through)
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

### Core/Inc/cansult_diag.h

Shared diagnostic counters. ISR-written fields are `volatile`. Mix of u16 (where wraparound during a normal session matters — `uart_ore/fe/ne`, `can_tx_fail`, `good_frame`, `implausible_frame`) and u8 (for rare events — `dma_restart`, `watchdog_timeout`, `reconnect`, `can_recover`). All wide-word writes are single-instruction on Cortex-M3 when aligned, so the ISR/main race is safe for diagnostics (look at deltas).

### Core/Src/cansult.c (HAL glue)

- DMA circular RX → ring buffer drain (polls `__HAL_DMA_GET_COUNTER`)
- Non-blocking `readEcuByte()`: drain + pop
- Blocking TX with 5ms timeout (safe during DMA RX — separate HAL gState/RxState)
- `stopStream()`: sends 0x30 ×3, waits 200ms for 0xCF ACK, flushes ring buffer
- `requestStreaming()`: batch TX 35 bytes in single HAL call
- DMA auto-restart: if `huart1.RxState == READY`, re-init DMA (increments `dma_restart_count`)
- `canTx()`: helper wrapping `HAL_CAN_AddTxMessage` with mailbox check
- `sendDiagFrame()`: 1Hz diagnostics — 0x665 (state + rates + freshness) and 0x66B (u16 error counters + MCU temp + mode/CAN state)
- `recoverCan()`: brings the CAN peripheral back from bus-off / RESET / ERROR via `HAL_CAN_Stop` + deinit/init/start (bumps `can_recover_count`)
- `enterAdapterMode()` / `exitAdapterMode()`: toggled by CAN 0x66D. Adapter mode releases PA9 to high-Z, powers BT_EN (PA8), and pauses parser/watchdog/CAN stream so the PC can talk straight to the ECU over BT
- Debug UART stream: raw RX/TX bytes on 0x669/0x66A (off by default)
- CAN RX: filter for 0x66F (debug cmd) + 0x66D (mode cmd), polled in `cansult_tick()`
- Race fix: `processEcuBytes()` runs before `route()` so watchdog sees freshly drained bytes
- Plausibility guard: `consult_parser_validate_stream()` rejects frames with impossible values (RPM>10000, speed>250, etc.), rolls `data[]` back to last-good snapshot; counter `implausible_frame_count`

### Core/Src/stm32f1xx_it.c (ISR)

- `DMA1_Channel5_IRQHandler` — HAL DMA handler (CubeMX generated)
- `USART1_IRQHandler` — IDLE line detection (sets flag for main loop), clears ORE/FE/NE error flags and increments diagnostic counters

## UART/DMA Configuration

Configured via CubeMX (`cansult.ioc`):

| Parameter | Value |
|-----------|-------|
| USART1 Baud | 9600 |
| DMA1 Channel | 5 (USART1_RX) |
| DMA Mode | Circular |
| DMA Priority | High |
| DMA Buffer | 128 bytes |
| Ring Buffer | 128 bytes |
| NVIC DMA1_Ch5 | Priority 1 |
| NVIC USART1 | Priority 2 |
| Stack | 0x800 (2KB) |

## CAN Protocol

See `README.md` for frame layout and data conversion.

- CAN IDs: 0x666-0x668 (data), 0x665/0x66B (diagnostics @ 1Hz), 0x669/0x66A (debug stream), 0x66F (debug cmd RX), 0x66D (mode cmd RX)
- Bitrate: 500 kbit/s
- TX rate: 20Hz (50ms interval), paused in ADAPTER mode (BT pass-through)
- 17 ECU registers streamed per cycle

## Consult Protocol

See `docs/UART_RELIABILITY.md` for detailed protocol analysis.

- Init: `0xFF 0xFF 0xEF` (sent twice), ECU responds `0x10`
- Stream request: `0x5A <reg> ... 0xF0`, ECU streams `0xFF <count=17> <data...>` continuously
- Stop: `0x30`, ECU responds `0xCF`
- Baud: 9600 8N1

## Testing

28 host tests via Unity (`make test`):

- `test_uart_rx_buf` (7 tests): push/pop, FIFO order, wraparound, full drops, flush
- `test_consult_parser` (21 tests): init response, stream frames, ECU info parsing (24-byte layout), fault codes (incl. bounds check, zero count), state guards, plausibility check (accept/reject/rollback across RPM/speed/battery/MAF, first-bad-with-no-snapshot, engine-off)
