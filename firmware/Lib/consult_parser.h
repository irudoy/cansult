#ifndef CONSULT_PARSER_H
#define CONSULT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol constants */
#define CONSULT_COMMAND_INIT         0xEF
#define CONSULT_COMMAND_READ_REGISTER 0x5A
#define CONSULT_COMMAND_SELF_DIAG    0xD1
#define CONSULT_COMMAND_CLEAR_CODES  0xC1
#define CONSULT_COMMAND_ECU_INFO     0xD0
#define CONSULT_COMMAND_TERM         0xF0
#define CONSULT_COMMAND_STOP_STREAM  0x30
#define CONSULT_REGISTER_NULL        0xFF

#define CONSULT_STREAM_FRAME_SIZE   17
#define CONSULT_ECU_INFO_FRAME_SIZE 24
#define CONSULT_ECU_PART_NO_OFFSET  19
#define CONSULT_ECU_PART_NO_PREFIX_LEN 6
#define CONSULT_FAULT_CODES_BUF_SIZE 64

/* Parser state (matches existing cansult states) */
typedef enum {
    CONSULT_STATE_STARTUP = 0,
    CONSULT_STATE_INITIALIZING,
    CONSULT_STATE_POST_INIT,
    CONSULT_STATE_WAITING_ECU_RESPONSE,
    CONSULT_STATE_IDLE,
    CONSULT_STATE_STREAMING,
} consult_state_t;

/* Frame read sub-state */
typedef enum {
    CONSULT_FRSTATE_STREAM = 0,
    CONSULT_FRSTATE_ECU_INFO,
    CONSULT_FRSTATE_FAULT_CODES,
} consult_frame_state_t;

/* Events returned by consult_parser_feed */
typedef enum {
    CONSULT_EVENT_NONE = 0,
    CONSULT_EVENT_INIT_OK,           /* ECU init response received */
    CONSULT_EVENT_ECU_INFO_READY,    /* ECU part number parsed */
    CONSULT_EVENT_STREAM_FRAME,      /* Stream data frame ready in data[] */
    CONSULT_EVENT_FAULT_CODES_READY, /* Fault codes parsed */
    CONSULT_EVENT_CLEAR_CODES_OK,    /* Clear DTC acknowledged */
} consult_event_t;

typedef struct {
    /* Protocol state */
    consult_state_t state;

    /* Frame reader */
    consult_frame_state_t frame_state;
    bool reading_frame;
    uint8_t frame_count;

    /* Byte tracking */
    uint8_t curr_byte;
    uint8_t prev_byte;

    /* Stream data (17 registers) */
    uint8_t data[CONSULT_STREAM_FRAME_SIZE];

    /* ECU part number: "23710-XXXXX" */
    char ecu_part_no[11];

    /* Fault codes */
    uint8_t fault_codes_count;
    uint8_t fault_codes[CONSULT_FAULT_CODES_BUF_SIZE];
} consult_parser_t;

/* Initialize parser to startup state */
void consult_parser_init(consult_parser_t *p);

/* Feed one byte from ECU. Returns event if a complete unit was parsed. */
consult_event_t consult_parser_feed(consult_parser_t *p, uint8_t byte);

/* Set parser state (called by HAL layer on state transitions) */
void consult_parser_set_state(consult_parser_t *p, consult_state_t state);

/* Start frame reader for a given sub-state */
void consult_parser_start_frame(consult_parser_t *p, consult_frame_state_t fstate);

/* Reset frame reader */
void consult_parser_reset_frame(consult_parser_t *p);

#endif
