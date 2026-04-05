#include "consult_parser.h"

static bool error_check_command_byte(uint8_t cmd, uint8_t check) {
    return cmd == (uint8_t)~check;
}

void consult_parser_init(consult_parser_t *p) {
    p->state = CONSULT_STATE_STARTUP;
    p->frame_state = CONSULT_FRSTATE_STREAM;
    p->reading_frame = false;
    p->frame_count = 0;
    p->curr_byte = 0;
    p->prev_byte = 0;

    for (int i = 0; i < CONSULT_STREAM_FRAME_SIZE; i++)
        p->data[i] = 0;

    p->ecu_part_no[0] = '2';
    p->ecu_part_no[1] = '3';
    p->ecu_part_no[2] = '7';
    p->ecu_part_no[3] = '1';
    p->ecu_part_no[4] = '0';
    p->ecu_part_no[5] = '-';
    for (int i = 6; i < 11; i++)
        p->ecu_part_no[i] = '\0';

    p->fault_codes_count = 0;
    for (int i = 0; i < CONSULT_FAULT_CODES_BUF_SIZE; i++)
        p->fault_codes[i] = 0xFF;
}

void consult_parser_set_state(consult_parser_t *p, consult_state_t state) {
    p->state = state;
}

void consult_parser_start_frame(consult_parser_t *p, consult_frame_state_t fstate) {
    p->frame_state = fstate;
    p->reading_frame = true;
    p->frame_count = 0;
}

void consult_parser_reset_frame(consult_parser_t *p) {
    p->reading_frame = false;
    p->frame_count = 0;
    p->frame_state = CONSULT_FRSTATE_STREAM;
}

consult_event_t consult_parser_feed(consult_parser_t *p, uint8_t byte) {
    p->prev_byte = p->curr_byte;
    p->curr_byte = byte;

    /* Frame reader active — consume bytes into the appropriate buffer */
    if (p->reading_frame) {
        switch (p->frame_state) {

        case CONSULT_FRSTATE_STREAM:
            p->data[p->frame_count] = byte;
            p->frame_count++;
            if (p->frame_count == CONSULT_STREAM_FRAME_SIZE) {
                consult_parser_reset_frame(p);
                return CONSULT_EVENT_STREAM_FRAME;
            }
            return CONSULT_EVENT_NONE;

        case CONSULT_FRSTATE_ECU_INFO:
            if (p->frame_count >= CONSULT_ECU_PART_NO_OFFSET) {
                uint8_t idx = p->frame_count - CONSULT_ECU_PART_NO_OFFSET
                              + CONSULT_ECU_PART_NO_PREFIX_LEN;
                if (idx < 11)
                    p->ecu_part_no[idx] = (char)byte;
            }
            p->frame_count++;
            if (p->frame_count == CONSULT_ECU_INFO_FRAME_SIZE) {
                consult_parser_reset_frame(p);
                return CONSULT_EVENT_ECU_INFO_READY;
            }
            return CONSULT_EVENT_NONE;

        case CONSULT_FRSTATE_FAULT_CODES:
            if (p->prev_byte == CONSULT_REGISTER_NULL) {
                p->fault_codes_count = byte;
                /* Bounds check */
                if (p->fault_codes_count > CONSULT_FAULT_CODES_BUF_SIZE)
                    p->fault_codes_count = CONSULT_FAULT_CODES_BUF_SIZE;
                if (p->fault_codes_count == 0) {
                    consult_parser_reset_frame(p);
                    return CONSULT_EVENT_FAULT_CODES_READY;
                }
            } else if (byte != CONSULT_REGISTER_NULL) {
                if (p->frame_count < CONSULT_FAULT_CODES_BUF_SIZE)
                    p->fault_codes[p->frame_count] = byte;
                p->frame_count++;
                if (p->frame_count == p->fault_codes_count) {
                    consult_parser_reset_frame(p);
                    return CONSULT_EVENT_FAULT_CODES_READY;
                }
            }
            return CONSULT_EVENT_NONE;
        }
        return CONSULT_EVENT_NONE;
    }

    /* Not reading a frame — check for state transitions */

    /* Init OK: ECU responds with bitwise inverse of INIT command */
    if (p->state == CONSULT_STATE_INITIALIZING &&
        error_check_command_byte(byte, CONSULT_COMMAND_INIT)) {
        p->state = CONSULT_STATE_POST_INIT;
        return CONSULT_EVENT_INIT_OK;
    }

    /* Waiting for ECU response to a command */
    if (p->state == CONSULT_STATE_WAITING_ECU_RESPONSE) {
        if (error_check_command_byte(byte, CONSULT_COMMAND_ECU_INFO)) {
            consult_parser_start_frame(p, CONSULT_FRSTATE_ECU_INFO);
            return CONSULT_EVENT_NONE;
        }
        if (error_check_command_byte(byte, CONSULT_COMMAND_SELF_DIAG)) {
            consult_parser_start_frame(p, CONSULT_FRSTATE_FAULT_CODES);
            /* Reset fault codes buffer */
            p->fault_codes_count = 0;
            for (int i = 0; i < CONSULT_FAULT_CODES_BUF_SIZE; i++)
                p->fault_codes[i] = 0xFF;
            return CONSULT_EVENT_NONE;
        }
        if (error_check_command_byte(byte, CONSULT_COMMAND_CLEAR_CODES)) {
            return CONSULT_EVENT_CLEAR_CODES_OK;
        }
    }

    /* Streaming: look for frame sync (0xFF followed by frame size) */
    if (p->state == CONSULT_STATE_STREAMING) {
        if (byte == CONSULT_STREAM_FRAME_SIZE &&
            p->prev_byte == CONSULT_REGISTER_NULL) {
            consult_parser_start_frame(p, CONSULT_FRSTATE_STREAM);
        }
    }

    return CONSULT_EVENT_NONE;
}
