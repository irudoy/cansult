# Roadmap

## Hardware

### Next PCB revision

- [ ] **Remove consult clock generator chain** in `hardware/consult.kicad_sch`: `U2` (MC74HC4060 divider), the 4.9152 MHz crystal, both 30p load caps, the 1n decoupling cap, the 10M feedback resistor, and the 4K7 enable pull-up on U2 — MCU now generates the 153.52 kHz consult CLK in firmware via TIM2_CH2 PWM. The external divider was redundant and its LMV358/LDO heating was the source of thermal UART failures above ~28.5 °C (see `docs/UART_RELIABILITY.md`).
- [ ] **Route CLK pin → Q3 gate directly on PCB.** Currently a bodge wire from `J2` pin 6 (SWO = PB3) → former U2 pad 5 → JP1 → Q3 gate. Replace with a proper trace and drop JP1 (the Q4/Q5 baud selector is no longer meaningful — baud is chosen in firmware).
- [ ] **Drop JP1** (Q4/Q5 → CLK_O solder jumper). Dead after U2 removal.
- [ ] **Move CLK from PB3 to PA1 (TIM2_CH2 native).** PB3 was chosen ad-hoc because SWO was unused; moving CLK off PB3 frees SWO for debug. PA1 gives TIM2_CH2 natively (no remap), so firmware keeps using TIM2_CH2 — just drop the AFIO partial remap and change the pin init from PB3 to PA1. PA1 is also on the U4 side that faces the Consult interface (pin 8 on QFN, left edge), so routing to Q3 should be shorter than the current PB3 path (pin 30, top edge).
- [ ] **Status LED(s).** At least one user-facing LED on a free GPIO (heartbeat / consult-link / error). Currently no visibility into device state without CAN bus or SWD. Plenty of free pins available: PA0, PA4–PA7, PB0–PB2, PA15, PB4–PB7 (14 free after PA1 is taken by CLK).
- [ ] **Spare programming / debug header.** Current SWD is on `J2`; add a second small header or exposed pads for serial console (USART2 on PA2/PA3 is wired but unused externally — see note below) or spare GPIO breakout so bench debugging doesn't require holding a wire to U4 pads.
- [ ] **Verify & debug Bluetooth extension.** `BT_EN` (PA8) is reserved and `J4` BT connector exists, but the BT expansion board itself has not been tested end-to-end. Validate pinout, enable/wake sequencing, and UART routing before committing the next PCB.
- [ ] **Investigate USART1/USART2 net sharing.** Schematic has `MCU_TX`/`MCU_RX` nets connected to *both* PA9/PA10 (USART1) and PA2/PA3 (USART2) on U4 — two TX pins tied to the same output net. Harmless only if USART2 is left as GPIO high-Z in firmware, but this is fragile. In the next rev, break the PA2/PA3 connection and either route USART2 to the spare debug header (above) or leave the pads unpopulated.

### Rework on current PCB (done on this board)

- [x] U2, Y?, C? desoldered.
- [x] Wire from `J2` pin 6 (PB3) to U2 pad 5 (CLK_Q5) — routes via existing JP1 (bridged to Q5) to Q3 gate.

## Firmware

- [ ] **BT adapter mode — e2e validation** (implemented 2026-04-24, commit df8ba0f). Firmware side done: CAN 0x66D toggles `cansult_mode_t` between STREAM (normal 20 Hz CAN TX) and ADAPTER (stopStream → PA9 high-Z → BT_EN high → parser paused); DIAG2 byte 7 bit 7 reflects mode. Not yet validated on hardware: need a real PC ↔ BC417 ↔ ECU session (Nistune / NDSIII) and confirmation that (a) the ECU actually halts its telemetry after `stopStream()` before PA9 goes high-Z, (b) PA9 high-Z doesn't back-drive the shared BT_TX line, (c) `exitAdapterMode` resyncs the DMA/parser without losing the initial ECU handshake. Ties into the hardware-side "Verify & debug Bluetooth extension" item above.

## Done

- [x] **Stream plausibility check** (2026-04-20). After a 17-byte CONSULT_FRSTATE_STREAM frame is assembled, `consult_parser_validate_stream()` applies per-field range limits (RPM ≤ 10000 rpm, Speed ≤ 250 km/h, INJ ≤ 100 ms, MAF ≤ 5.115 V, Battery 3.2–20 V). On failure, `data[]` is rolled back to the last plausible snapshot so CAN TX repeats the previous known-good frame instead of emitting a UART-shift spike (observed RPM 745 663 in canlogger log-19-04). New counter `cansult_diag.implausible_frame_count`. See `Lib/consult_parser.c:consult_parser_validate_stream` and `test/test_consult_parser.c` (7 new unit tests).
