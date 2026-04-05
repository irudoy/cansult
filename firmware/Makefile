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

# --- Build ---

.PHONY: build clean test flash erase reset debug gdb-server

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
	openocd -f interface/stlink.cfg -c "hla_serial $(STLINK_SN)" -f target/stm32f1x.cfg

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

# --- Debug (ST-LINK GDB server) ---

gdb-server:
	$(GDB_SERVER) -p 3333 -d -e -cp $(dir $(PROGRAMMER))

debug: build
	$(GDB) $(ELF) \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"
