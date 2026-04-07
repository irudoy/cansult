printf "=== Parser ===\n"
printf "state=%d frame_state=%d reading=%d frame_count=%d\n", parser.state, parser.frame_state, parser.reading_frame, parser.frame_count
printf "curr=0x%02X prev=0x%02X\n", parser.curr_byte, parser.prev_byte
printf "data: %02X %02X %02X %02X %02X %02X %02X %02X\n", parser.data[0], parser.data[1], parser.data[2], parser.data[3], parser.data[4], parser.data[5], parser.data[6], parser.data[7]

printf "=== Diagnostics ===\n"
printf "ORE=%d FE=%d NE=%d CAN_fail=%d DMA_rst=%d WDT=%d good_frames=%d\n", cansult_diag.uart_ore_count, cansult_diag.uart_fe_count, cansult_diag.uart_ne_count, cansult_diag.can_tx_fail_count, cansult_diag.dma_restart_count, cansult_diag.watchdog_timeout_count, cansult_diag.good_frame_count

printf "=== DMA / UART ===\n"
printf "dmaRxReadPos=%d uartIdleFlag=%d\n", dmaRxReadPos, uartIdleFlag
printf "rxBuf head=%d tail=%d\n", rxBuf.head, rxBuf.tail
printf "debugStream=%d\n", debugStream

printf "=== Timing ===\n"
printf "time=%u tempTime=%u lastFrameSent=%u lastDiag=%u\n", time, tempTime, lastFrameSentTime, lastDiagTime
printf "delta_temp=%u delta_frame=%u delta_diag=%u\n", time-tempTime, time-lastFrameSentTime, time-lastDiagTime

printf "=== USART1 Registers ===\n"
printf "SR=0x%08X DR=0x%02X BRR=0x%08X CR1=0x%08X\n", *(uint32_t*)0x40013800, *(uint32_t*)0x40013804, *(uint32_t*)0x40013808, *(uint32_t*)0x4001380C

printf "=== DMA1 Ch5 (USART1_RX) ===\n"
printf "CCR=0x%08X CNDTR=%d CPAR=0x%08X CMAR=0x%08X\n", *(uint32_t*)0x40020058, *(uint32_t*)0x4002005C, *(uint32_t*)0x40020060, *(uint32_t*)0x40020064

printf "=== CAN Registers ===\n"
printf "ESR=0x%08X TSR=0x%08X RF0R=0x%08X BTR=0x%08X\n", *(uint32_t*)0x40006418, *(uint32_t*)0x40006408, *(uint32_t*)0x4000640C, *(uint32_t*)0x4000641C

printf "=== Backtrace ===\n"
bt 5
