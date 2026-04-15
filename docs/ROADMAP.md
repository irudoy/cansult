# Roadmap

## Hardware

### Next PCB revision

- **Remove consult clock generator chain (U2 MC74HC4060 + crystal Y? + load caps C?)** — MCU now generates the 153.52 kHz consult CLK in firmware via TIM2_CH2 PWM on PB3. The external divider is redundant and was the source of thermal UART failures above ~28.5 °C (see commit history).
- **Route CLK pin → Q3 gate directly on PCB.** Currently a bodge wire from `J2` pin 6 (SWO = PB3) to the former U2 pad 5 carries the MCU-generated clock through JP1 to Q3. In the next rev, replace with a proper trace and drop JP1 (Q4/Q5 baud selector is no longer meaningful — baud is chosen in firmware).
- **Drop JP1** (Q4/Q5 → CLK_O solder jumper). Dead after U2 removal.
- **Reconsider which MCU pin drives CLK.** PB3 was chosen ad-hoc because SWO was unused. Revisit in next rev — pick the pin that routes cleanest to Q3 gate, with timer capability (TIM1/2/3/4 CH). Freeing PB3 would let us keep SWO as a debug option.
- **Status LED(s).** At least one user-facing LED driven from a free GPIO (heartbeat / consult-link / error). Optional second for CAN activity. Currently no visibility into device state without CAN bus or SWD.
- **Spare programming / debug header.** Current SWD is on `J2`; add a second small header or exposed pads for serial console or spare GPIO breakout so bench debugging doesn't require holding a wire to U4 pads.
- **Verify & debug Bluetooth extension.** `BT_EN` (PA8) is reserved but the BT expansion board itself has not been tested end-to-end. Validate pinout, enable/wake sequencing, and UART routing before committing the next PCB.

### Rework on current PCB (done on this board)

- U2, Y?, C? desoldered.
- Wire from `J2` pin 6 (PB3) to U2 pad 5 (CLK_Q5) — routes via existing JP1 (bridged to Q5) to Q3 gate.

## Firmware

- **CAN auto-recovery.** Peripheral has no recovery from bus-off / `HAL_CAN_STATE_RESET` (observed 2026-04-15 after a heat-gun-induced brownout — SRAM/UART state survived, but CAN state registers cleared and firmware did not re-init). Needs `HAL_CAN_ErrorCallback` + periodic state check in `cansult_tick`, plus reconciling `cansult.ioc` `AutoBusOff=ENABLE` vs generated `main.c` `AutoBusOff=DISABLE`. Not bundled with the thermal fix — separate focused change.
