# CubeIDE paths
CUBEIDE      := /Applications/STM32CubeIDE.app/Contents/MacOS/STM32CubeIDE
CUBE_PLUGINS := /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins
PROGRAMMER   := $(CUBE_PLUGINS)/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_1.0.0.202601242230/tools/bin/STM32_Programmer_CLI
GDB_SERVER   := $(CUBE_PLUGINS)/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.macosaarch64_1.0.0.202601242230/tools/bin/ST-LINK_gdbserver
GDB          := $(CUBE_PLUGINS)/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin/arm-none-eabi-gdb

PROJECT      := cansult
CONFIG       := Debug
PROJECT_DIR  := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
ELF          := $(PROJECT_DIR)/Debug/$(PROJECT).elf
WS           := /tmp/cansult-ws
STLINK_SN    := 210041000B000057334A534E

# --- Help ---

list:
	@echo "Build:    build clean test"
	@echo "Flash:    flash erase reset"
	@echo "Debug:    ocd-server ocd-status ocd-dump gdb-read gdb-exec"
	@echo "CAN:      can-monitor can-diag can-debug-on can-debug-off can-adapter-on can-adapter-off"
	@echo "Capture:  can-capture can-capture-diag can-capture-debug"

# --- Build ---

.PHONY: build clean test flash erase reset debug gdb-server gdb-read gdb-exec \
       ocd-dump can-monitor can-diag can-debug-on can-debug-off \
       can-adapter-on can-adapter-off \
       can-capture can-capture-diag can-capture-debug

build:
	$(CUBEIDE) --launcher.suppressErrors -nosplash \
		-application org.eclipse.cdt.managedbuilder.core.headlessbuild \
		-data $(WS) \
		-import $(PROJECT_DIR) \
		-build $(PROJECT)/$(CONFIG)

clean:
	$(CUBEIDE) --launcher.suppressErrors -nosplash \
		-application org.eclipse.cdt.managedbuilder.core.headlessbuild \
		-data $(WS) \
		-import $(PROJECT_DIR) \
		-cleanBuild $(PROJECT)/$(CONFIG)

# --- Test (host) ---

test:
	$(MAKE) -C test

# --- Flash & Reset ---

flash: build
	$(PROGRAMMER) -c port=SWD sn=$(STLINK_SN) -w $(ELF) -v -rst

erase:
	$(PROGRAMMER) -c port=SWD sn=$(STLINK_SN) -e all

reset:
	$(PROGRAMMER) -c port=SWD sn=$(STLINK_SN) -rst

# --- Debug (OpenOCD) ---

ocd-server:
	openocd -f interface/stlink.cfg -c "adapter serial $(STLINK_SN)" -f target/stm32f1x.cfg

ocd-debug: build
	$(GDB) $(ELF) \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"

ocd-status:
	$(GDB) $(ELF) -batch \
		-ex "target extended-remote :3333" \
		-ex "monitor reset run" \
		-ex "shell sleep 3" \
		-ex "monitor halt" \
		-ex "printf \"=== Parser ===\n\"" \
		-ex "printf \"state=%d frame_state=%d reading=%d count=%d\n\", parser.state, parser.frame_state, parser.reading_frame, parser.frame_count" \
		-ex "printf \"curr=0x%02X prev=0x%02X\n\", parser.curr_byte, parser.prev_byte" \
		-ex "printf \"data: %02X %02X %02X %02X %02X %02X %02X %02X\n\", parser.data[0], parser.data[1], parser.data[2], parser.data[3], parser.data[4], parser.data[5], parser.data[6], parser.data[7]" \
		-ex "printf \"=== DMA ===\n\"" \
		-ex "printf \"rxBuf avail: head=%d tail=%d\n\", rxBuf.head, rxBuf.tail" \
		-ex "printf \"dmaReadPos=%d\n\", dmaRxReadPos" \
		-ex "printf \"uartIdleFlag=%d\n\", uartIdleFlag" \
		-ex "bt" \
		-ex "monitor resume"

# --- Debug (GDB batch helpers) ---

# Read variables/expressions via GDB. Auto-starts and stops OpenOCD.
# Usage: make gdb-read EXPRS="parser.state dmaRxReadPos uartIdleFlag"
#        make gdb-read EXPRS="*(uint32_t*)0x40013800"
gdb-read:
	@openocd -f interface/stlink.cfg -c "adapter serial $(STLINK_SN)" -f target/stm32f1x.cfg > /tmp/openocd.log 2>&1 & \
	OCD_PID=$$!; sleep 1; \
	$(GDB) $(ELF) -batch \
		-ex "target extended-remote :3333" \
		-ex "monitor halt" \
		$(foreach e,$(EXPRS),-ex "print $(e)") \
		-ex "monitor resume" 2>&1; \
	kill $$OCD_PID 2>/dev/null; wait $$OCD_PID 2>/dev/null; true

# Run arbitrary GDB commands from a script file. Auto-starts and stops OpenOCD.
# Usage: make gdb-exec SCRIPT=scripts/dump.gdb
#    or: echo -e "bt\ninfo registers" > /tmp/cmds.gdb && make gdb-exec SCRIPT=/tmp/cmds.gdb
gdb-exec:
	@openocd -f interface/stlink.cfg -c "adapter serial $(STLINK_SN)" -f target/stm32f1x.cfg > /tmp/openocd.log 2>&1 & \
	OCD_PID=$$!; sleep 1; \
	$(GDB) $(ELF) -batch \
		-ex "target extended-remote :3333" \
		-ex "monitor halt" \
		-x "$(SCRIPT)" \
		-ex "monitor resume" 2>&1; \
	kill $$OCD_PID 2>/dev/null; wait $$OCD_PID 2>/dev/null; true

# Full state dump via GDB script
ocd-dump:
	$(MAKE) gdb-exec SCRIPT=scripts/dump.gdb

# --- CAN monitor (requires python-can + PCAN) ---

PCAN_CHANNEL ?= PCAN_USBBUS1
CAN_BITRATE  ?= 500000

can-monitor:
	python3 scripts/can_monitor.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) --stdout

can-diag:
	python3 scripts/can_monitor.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) --stdout --ids 0x665,0x66B

# Send CAN command to toggle debug UART stream
can-debug-on:
	python3 scripts/can_send.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) 0x66F 01

can-debug-off:
	python3 scripts/can_send.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) 0x66F 00

# Enter BT pass-through adapter mode (PC-side Nistune/NDSIII talks to ECU via BT)
can-adapter-on:
	python3 scripts/can_send.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) 0x66D 01

# Exit adapter mode back to 20Hz CAN stream
can-adapter-off:
	python3 scripts/can_send.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) 0x66D 00

# --- CAN capture (atomic: open, capture N seconds, close) ---

CAN_DURATION ?= 3

# Capture all cansult frames
# Usage: make can-capture DURATION=5
can-capture:
	python3 scripts/can_capture.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) --duration $(CAN_DURATION)

# Capture diagnostics only (1Hz 0x665)
# Usage: make can-capture-diag DURATION=3
can-capture-diag:
	python3 scripts/can_capture.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) --duration $(CAN_DURATION) --ids 0x665,0x66B

# Enable debug stream + capture UART traffic
# Usage: make can-capture-debug DURATION=5
can-capture-debug:
	python3 scripts/can_capture.py --channel $(PCAN_CHANNEL) --bitrate $(CAN_BITRATE) --duration $(CAN_DURATION) --ids 0x665,0x669,0x66A --pre-cmd "0x66F 01"

# --- Debug (ST-LINK GDB server) ---

gdb-server:
	$(GDB_SERVER) -p 3333 -d -e -cp $(dir $(PROGRAMMER))

debug: build
	$(GDB) $(ELF) \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"
