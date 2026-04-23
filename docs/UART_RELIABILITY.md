# UART Reliability Research: cansult ECU Communication

Research date: 2026-04-06.

> **Status (updated 2026-04-24).** Kept for historical context. The
> "root cause still unknown" note in §6 has since been resolved:
> the periodic drops were **thermal** — the LDO/LMV358 chain on the
> CLK path heated above ≈28.5 °C and corrupted UART timing. Fix:
> MCU now drives the consult CLK via TIM2_CH2 PWM on PB3 (commit
> 548cc15), and the U2 MC74HC4060 divider is being removed in the
> next PCB rev (see `docs/ROADMAP.md`). All Level 3 (CAN robustness)
> and Level 4 (observability) items below have been implemented;
> see updated checkboxes at the end of this file.

## Problem Statement

Known issue: cansult UART<->ECU connection drops after some time during streaming.
The cansult board (STM32F103) communicates with Nissan ECU via UART 9600 8N1, maps ECU registers to CAN frames, and sends them to canlogger at 20Hz.

---

## 1. Nissan Consult Protocol Behavior

Sources:
- `docs/Consult_Protocol_&_Commands_Issue_6.pdf` (local)
- [Consult Protocol Issue 5](https://evoecu.logic.net/mirror/nissan/Consult_Protocol_&_Commands_Issue_5.pdf)
- [Consult Protocol Issue 7 (Scribd)](https://www.scribd.com/document/146458636/Consult-Protocol-Commands-Issue-7)

### Initialization

- Sequence: `0xFF 0xFF 0xEF` at 9600 baud
- ECU responds with `0x10` (bitwise inversion of `0xEF`)
- Sometimes needs to be sent **twice** (confirmed empirically, firmware already does this)
- Some ECUs use different init bytes: `0xED` (A32 OBD2), `0xEA` (A/T), `0xE4` (S-HICAS), `0xDF` (Aircon)
- **Idle timeout after init: >30 minutes** -- re-init only needed after ECU power loss

### Streaming Mode

- Request: `0x5A <reg1> 0x5A <reg2> ... 0xF0`
- ECU ACKs each register: `0xA5 <reg_no>` (or `0xA5 0xFE` if unsupported)
- ECU then streams continuously: `0xFF <byte_count> <data_bytes>` repeating
- **ECU does NOT have a reader watchdog** -- it streams at its own pace indefinitely
- Maximum 20 bytes per stream response frame

### Stop Command

- **Spec says `0x30`**, cansult firmware uses `0x03` -- potential discrepancy
- ECU finishes current frame, then sends Stop ACK `0xCF`
- Caller should read until `0xCF` to maintain sync

### Key Insight

The ECU itself does not drop the connection. It streams indefinitely once started.
**All communication failures originate on the reader side** (byte loss, overrun, framing errors).

---

## 2. Root Cause Analysis: 14 Failure Modes

Identified by deep analysis of `Core/Src/cansult.c`, `Core/Src/main.c`, `Core/Src/stm32f1xx_it.c`, `Core/Src/stm32f1xx_hal_msp.c`.

### Critical (root cause of drops)

| # | Issue | Details |
|---|-------|---------|
| 1 | **Blocking UART RX polling, 1s timeout** | `HAL_UART_Receive(&huart1, &rxEcuByte, 1, 1000)` in `readEcu()`. Each `cansult_tick()` blocks up to 1 second if no byte available. At 9600 baud, bytes arrive every ~1.04ms. |
| 2 | **No UART RX interrupt, no DMA** | No `USART1_IRQn` in interrupt table, no DMA configured in MSP init. STM32F103 USART has **1-byte RX register** -- no hardware FIFO. Any processing delay >1ms = overrun. |
| 3 | **`requestStreaming()` sends 35 blocking TX bytes** | 17 registers x 2 bytes + 1 term = 35 bytes. At 9600 baud ~36ms of blocking TX. During this time, any ECU response bytes are lost (no RX interrupt). |
| 4 | **`stopStream()` blocks with `HAL_Delay(100)`** | 100ms hard delay after sending stop command, then unbounded drain loop where each failed read blocks 1s. |

### High

| # | Issue | Details |
|---|-------|---------|
| 5 | **`WAITING_ECU_RESPONSE` has no timeout** | If ECU response is lost (bytes dropped), firmware hangs in this state forever. No watchdog like in INITIALIZING/STREAMING states. |
| 6 | **Fault codes buffer overflow** | `currentFaultCodesCount` is set from ECU byte with no bounds check. Buffer is 64 bytes. If ECU reports count >64, writes past buffer. If count is 0, loops 256 times corrupting memory. |
| 7 | **Frame sync can be permanently lost** | Sync pattern `0xFF 0x11` can appear in data values, causing false sync. Lost bytes shift framing permanently until watchdog resets. |
| 8 | **CAN TX failures ignored** | `HAL_CAN_AddTxMessage` return value not checked. 3 frames sent in burst, only 3 TX mailboxes. `AutoBusOff = DISABLE` -- CAN bus-off is permanent. |

### Medium

| # | Issue | Details |
|---|-------|---------|
| 9 | **Torn reads of `data[]`** | CAN TX in `route()` reads `data[]` while frame reader may be mid-frame. Sends mix of old+new sample. |
| 10 | **`ecuPartNo` no null terminator** | If all 5 suffix bytes written, no trailing `\0`. |
| 11 | **`Error_Handler` hangs forever** | `__disable_irq(); while(1){}` -- no recovery, no indication. |

### Low

| # | Issue | Details |
|---|-------|---------|
| 12 | **`HAL_GetTick` overflow at 49.7 days** | Unsigned wraparound is safe for comparisons, but continuous operation >49 days untested. |
| 13 | **Transient state in debug CAN frames** | `stopStream()` sets IDLE, then callers overwrite. Debug trace shows spurious IDLE transitions. |

### Self-Reinforcing Failure Pattern

1. Main loop busy (CAN TX, blocking UART TX) for >1ms
2. UART byte arrives, no interrupt to capture it -> overrun (ORE flag set)
3. `HAL_UART_Receive` returns `HAL_ERROR` on subsequent calls (ORE not cleared)
4. 3-second watchdog fires -> `initECU()` -> `stopStream()` -> more blocking TX + `HAL_Delay(100)`
5. More bytes lost during recovery -> cycle repeats
6. UART peripheral stays in error state -> **permanent failure until power cycle**

---

## 3. Reference Implementations Comparison

### Arduino Consult Library ([Crim/Arduino-Nissan-Consult-Library](https://github.com/Crim/Arduino-Nissan-Consult-Library))

- Init timeout: **2 seconds** (longer than standard read)
- Init retries: **2 attempts**, 250ms between retries
- Read polling: **5ms interval** with `Serial.available()` check -- **never blocks**
- Stop delay: 100ms
- Error check: inverted command byte validation, `stopEcuStream()` on failure
- Comment: *"worried about the Arduino not being able to keep up with the data feed"*

### nissan-simple-car-computer ([local](../nissan-simple-car-computer/))

- **Stop-and-request model**: sends `0x30` (stop), then re-requests registers each cycle
- Watchdog: **200ms** -- immediate `init_ECU()` if no response
- `Serial.available() > 0` before every read -- zero blocking
- `Serial.flush()` + `delay(10)` before each request cycle
- Decodes response by sliding window on `last[30]` buffer

### openconsult ([jonsim/openconsult](https://github.com/jonsim/openconsult))

- C++ library, early stage
- PC-side implementation (USB-serial), not embedded

### Key Takeaways from References

1. **All working implementations use non-blocking reads** (`Serial.available()` or equivalent)
2. **Shorter timeouts** (200ms-2s vs our 3s) catch failures faster
3. **Stop-and-request** is simpler and more resilient than continuous streaming
4. **Nobody uses 1-second blocking reads** in the main loop

---

## 4. STM32 UART Reliability Best Practices

Sources:
- [MaJerle/stm32-usart-uart-dma-rx-tx](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx) -- definitive STM32 UART DMA guide
- [STM32 UART DMA Idle Detection](https://stm32world.com/wiki/STM32_UART_DMA_Idle_Detection)
- [STM32 Circular DMA with Timeout](https://akospasztor.com/projects/stm32-dma-uart/)
- [STM32 Tutorial: UART DMA RX](https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/)
- [UART DMA with IDLE Line Detection](https://controllerstech.com/uart-dma-with-idle-line-detection/)

### DMA Circular RX Pattern (recommended)

For STM32F103 (BluePill, same MCU as cansult):
- **DMA1 Channel 5** for USART1 RX
- Circular mode -- DMA never stops, wraps around automatically
- Buffer size: 64-128 bytes (at 9600 baud, 64 bytes = ~67ms of data)
- **3 event sources**: DMA Half-Transfer (HT), DMA Transfer-Complete (TC), UART IDLE line
- IDLE line interrupt fires after 1 frame time (~1.04ms at 9600) of silence
- Set DMA and UART interrupts to **same preemption priority** (no self-preemption)

### Processing Pattern

```c
// Called from DMA HT, DMA TC, and UART IDLE interrupts
void process_dma_rx(void) {
    size_t pos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (pos != old_pos) {
        if (pos > old_pos) {
            process_data(&dma_buf[old_pos], pos - old_pos);
        } else { // wraparound
            process_data(&dma_buf[old_pos], BUFFER_SIZE - old_pos);
            if (pos > 0) process_data(&dma_buf[0], pos);
        }
        old_pos = pos;
    }
}
```

### UART Overrun Recovery (if not using DMA)

- Check `__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)` before reads
- Clear with `__HAL_UART_CLEAR_OREFLAG(&huart1)` (STM32F1: read SR then DR)
- Or: full `HAL_UART_DeInit` + `HAL_UART_Init` for persistent errors
- STM32 HAL quirk: after ORE, `HAL_UART_Receive` returns `HAL_ERROR` forever until flag cleared

---

## 5. Fix Plan & Implementation Status

### Level 1 -- UART Reception (eliminates root cause) ✅ DONE

1. ✅ **DMA Circular RX** on DMA1 Channel 5, 64-byte buffer (CubeMX configured)
2. ✅ **IDLE line interrupt** -- sets flag, consumed by `readEcuByte()`
3. ✅ **128-byte ring buffer** (`Lib/uart_rx_buf.c`) between DMA and parser
4. ✅ **Non-blocking TX** -- blocking with 5ms timeout (safe during DMA RX)

### Level 2 -- Protocol & State Machine ✅ DONE

5. ✅ **Timeout on ALL states** -- `WAITING_ECU_RESPONSE` 2s, `INITIALIZING` 3s
6. ✅ **Reduced watchdog** -- `STREAMING` 500ms (was 3000ms)
7. ✅ **Fixed stop command** -- `0x30` (was `0x03`, typo from original code)
8. ✅ **Read until `0xCF`** (stop ACK) with 200ms timeout, sends stop ×3
9. ✅ **Bounds check** fault codes count clamped to buffer size

### Level 3 -- CAN & System Robustness (TODO)

10. **`AutoBusOff = ENABLE`** for CAN auto-recovery
11. **Check `HAL_CAN_AddTxMessage` return** -- skip on mailbox full
12. **`AutoRetransmission = ENABLE`** for reliable CAN delivery
13. **IWDG (Independent Watchdog)** -- last resort hardware reset

### Level 4 -- Observability (TODO)

14. Error counters (overruns, reinits, lost frames) on debug CAN ID `0x665`
15. State duration tracking for diagnosing slow transitions

### Additional changes made during implementation

- **Architecture refactor**: business logic extracted to `Lib/` (host-testable), HAL glue in `Core/Src/cansult.c`
- **consult_parser** module: protocol byte parser as pure state machine, 21 unit tests (incl. plausibility guard)
- **uart_rx_buf** module: ring buffer, 7 unit tests
- **Makefile + test infra**: build/test/flash targets, Unity framework, 28 total tests
- **Stack size**: increased 0x400 → 0x800 (HAL_UART_Transmit + DMA drain call depth)
- **Batch TX**: `requestStreaming()` sends 35 bytes in single HAL call (was 35 separate calls)
- **ST-Link serial**: pinned in Makefile to prevent cross-flashing

---

## 6. Observed Behavior After Level 1+2 Fixes (2026-04-06)

Data captured via canlogger USB CDC debug stream (`docs/debug_stream.txt`).

### Periodic ECU disconnect pattern

Connection drops every ~40-60 seconds, reconnect takes ~10 seconds:

```
[176] 0x640 0x666 0x667       ← normal, ~60 frames/sec
[177] 0x640 0x665(state=00)   ← ECU data lost, reconnect starts
[178]       0x665(state=01)   ← connecting
[183]       0x665(state=03)   ← negotiating
[185]       0x665(state=00)
[188]       0x665(state=05)   ← streaming resumed
[188] 0x640 0x666 0x667       ← data back, ~60 frames/sec
[217]                         ← same cycle again after ~40s
```

State transitions during reconnect: `05→00→01→00→01→03→00→03→04→05`

### Key observations

1. **Drop duration**: ~10-12 seconds each reconnect cycle
2. **Drop interval**: ~40-60 seconds of stable streaming between drops
3. **Frames during normal**: ~60/sec (3 CAN IDs × 20Hz = expected)
4. **Frames during reconnect**: ~20/sec (only 0x640 switchboard ADC)
5. **Level 1+2 fixes work**: reconnect succeeds every time (no permanent hang)
6. **Root cause still unknown**: ECU streams indefinitely (per spec), so something on cansult side triggers the drop

### TODO (status 2026-04-24)

- [x] Level 3 — AutoBusOff + software `recoverCan()` for RESET/ERROR states landed in commit 0f87011. TX-error handling via `canTx` mailbox check is in place. IWDG deferred (not needed once thermal root cause was addressed).
- [x] Level 4 — 0x665 carries parser state + per-second rates (ORE, implausible, CAN fail) + monotonic u8 counters + freshness. Extended 0x66B adds full-width FE/NE u16, MCU temp, CAN recover count, mode/state.
- [x] UART error counters (ORE/FE/NE) exposed on both 0x665 (rate) and 0x66B (u16).
- [x] DMA buffer is 128 bytes (circular, DMA1 Ch5). See `docs/ARCHITECTURE.md`.
- [x] CAN TX mailbox check implemented in `canTx()`; failures increment `can_tx_fail_count` instead of blocking the drain.
- [x] Root cause of periodic drops identified as thermal (LDO/LMV358 on CLK path above ≈28.5 °C). MCU now generates the 153.52 kHz consult CLK via TIM2_CH2 PWM (commit 548cc15); external divider slated for removal on next PCB.
- [x] Plausibility guard rejects decoded frames with impossible values (RPM>10000 etc.) and rolls `data[]` back to last-good snapshot (`consult_parser_validate_stream`, commit 2d9f05f).
