# CLAUDE.md

## Project Overview

Nissan Consult (14-pin) to CAN adapter. Reads ECU registers via UART 9600 8N1, streams as CAN frames at 20Hz. STM32F103TBU6, custom PCB.

## Docs

- `README.md` — CAN protocol, frame layout, data conversion formulas
- `docs/ARCHITECTURE.md` — Firmware modules, data flow, state machine, test strategy
- `docs/UART_RELIABILITY.md` — Research: root causes of UART drops, fix plan, reference implementations

## Repository Structure

```
firmware/
├── Lib/              # Pure C business logic (no HAL), host-testable
│   ├── consult_parser.*  # Consult protocol parser state machine
│   └── uart_rx_buf.*     # Byte ring buffer (DMA → parser)
├── Core/Src/         # HAL wrappers (CubeMX USER CODE markers)
│   ├── cansult.c     # Main glue: DMA drain, state routing, CAN TX
│   ├── main.c        # CubeMX init, main loop
│   └── stm32f1xx_it.c # ISR: DMA, USART1 IDLE/error clearing
├── Core/Inc/
│   ├── cansult.h     # Constants, CAN IDs, register defs
│   └── cansult_diag.h # Diagnostic counters (UART errors, CAN fails, DMA restarts)
├── test/             # Host unit tests (Unity), 28 tests
│   ├── unity/
│   ├── test_uart_rx_buf.c
│   └── test_consult_parser.c
├── scripts/
│   ├── can_monitor.py # CAN monitor/logger, foreground (python-can, PCAN)
│   ├── can_capture.py # Atomic CAN capture: open, record N sec, close
│   ├── can_send.py    # Send single CAN frame
│   └── dump.gdb       # Full GDB state snapshot
├── Makefile          # build/test/flash/ocd-*/can-* commands
└── cansult.ioc       # CubeMX config (DMA, NVIC, USART1, CAN)
```

## Commands (from `firmware/`)

```bash
make build        # CubeIDE headless build
make test         # Host unit tests (28 tests)
make flash        # Build + flash via ST-Link SWD
make reset        # Hardware reset
make ocd-server   # OpenOCD GDB server on :3333
make ocd-status   # 3s run, halt, dump parser/DMA state
make gdb-read EXPRS="parser.state"  # Read variables (auto OpenOCD)
make gdb-exec SCRIPT=script.gdb    # Run GDB commands (auto OpenOCD)
make ocd-dump         # Full state snapshot via GDB
make can-monitor      # Foreground CAN monitor (all frames, for human use)
make can-diag         # Foreground, 0x665 + 0x66B diagnostics only
make can-debug-on     # Enable UART debug stream (0x669/0x66A)
make can-debug-off    # Disable UART debug stream
make can-adapter-on   # Enter BT pass-through ADAPTER mode (0x66D)
make can-adapter-off  # Exit ADAPTER mode back to STREAM
make can-capture          # Atomic: capture all frames for N sec (CAN_DURATION=3)
make can-capture-diag     # Atomic: capture 0x665 + 0x66B only
make can-capture-debug    # Atomic: enable debug + capture UART traffic
```

## Testing

TDD. All business logic in `Lib/`, tested on host with gcc + Unity.

```bash
cd firmware && make test
```

## Key Conventions

- **Lib/** — no HAL, no CubeMX. Plain C structs, testable on host.
- **Core/Src/** — CubeMX `USER CODE BEGIN/END` markers. Custom code within markers only.
- CubeMX config changes → regenerate code, don't edit hal_msp.c/it.c outside markers.
- ST-Link serial pinned in Makefile (`STLINK_SN`).
