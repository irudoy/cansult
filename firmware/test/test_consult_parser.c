#include "unity.h"
#include "consult_parser.h"

static consult_parser_t p;

void setUp(void) { consult_parser_init(&p); }
void tearDown(void) {}

/* --- Init --- */

void test_init_defaults(void) {
    TEST_ASSERT_EQUAL(CONSULT_STATE_STARTUP, p.state);
    TEST_ASSERT_FALSE(p.reading_frame);
    TEST_ASSERT_EQUAL_STRING("23710-", p.ecu_part_no);
}

void test_init_response(void) {
    consult_parser_set_state(&p, CONSULT_STATE_INITIALIZING);

    /* ECU responds with bitwise inversion of 0xEF = 0x10 */
    consult_event_t ev = consult_parser_feed(&p, 0x10);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_INIT_OK, ev);
    TEST_ASSERT_EQUAL(CONSULT_STATE_POST_INIT, p.state);
}

void test_init_wrong_byte_ignored(void) {
    consult_parser_set_state(&p, CONSULT_STATE_INITIALIZING);

    consult_event_t ev = consult_parser_feed(&p, 0x42);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    TEST_ASSERT_EQUAL(CONSULT_STATE_INITIALIZING, p.state);
}

/* --- Stream frames --- */

void test_stream_frame_complete(void) {
    consult_parser_set_state(&p, CONSULT_STATE_STREAMING);

    /* Feed sync: 0xFF then 0x11 (17 = frame size) */
    consult_parser_feed(&p, 0xFF);
    consult_event_t ev = consult_parser_feed(&p, CONSULT_STREAM_FRAME_SIZE);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    TEST_ASSERT_TRUE(p.reading_frame);

    /* Feed 17 data bytes */
    for (uint8_t i = 0; i < CONSULT_STREAM_FRAME_SIZE - 1; i++) {
        ev = consult_parser_feed(&p, 0x10 + i);
        TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    }
    /* Last byte triggers frame complete */
    ev = consult_parser_feed(&p, 0x99);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_STREAM_FRAME, ev);
    TEST_ASSERT_FALSE(p.reading_frame);

    /* Verify data contents */
    TEST_ASSERT_EQUAL_HEX8(0x10, p.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x1F, p.data[15]);
    TEST_ASSERT_EQUAL_HEX8(0x99, p.data[16]);
}

void test_stream_sync_requires_0xff_prefix(void) {
    consult_parser_set_state(&p, CONSULT_STATE_STREAMING);

    /* Feed 0x11 without 0xFF prefix — should NOT start frame */
    consult_parser_feed(&p, 0x00);
    consult_parser_feed(&p, CONSULT_STREAM_FRAME_SIZE);
    TEST_ASSERT_FALSE(p.reading_frame);
}

void test_stream_0xff_in_data_no_false_sync(void) {
    consult_parser_set_state(&p, CONSULT_STATE_STREAMING);

    /* Start a valid frame */
    consult_parser_feed(&p, 0xFF);
    consult_parser_feed(&p, CONSULT_STREAM_FRAME_SIZE);
    TEST_ASSERT_TRUE(p.reading_frame);

    /* Feed 0xFF as data byte — should not restart frame */
    consult_parser_feed(&p, 0xFF);
    TEST_ASSERT_TRUE(p.reading_frame);
    TEST_ASSERT_EQUAL_UINT8(1, p.frame_count);
    TEST_ASSERT_EQUAL_HEX8(0xFF, p.data[0]);
}

/* --- ECU Info --- */

void test_ecu_info_parses_part_number(void) {
    consult_parser_set_state(&p, CONSULT_STATE_WAITING_ECU_RESPONSE);

    /* ECU responds with inverted ECU_INFO command: ~0xD0 = 0x2F */
    consult_event_t ev = consult_parser_feed(&p, 0x2F);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    TEST_ASSERT_TRUE(p.reading_frame);
    TEST_ASSERT_EQUAL(CONSULT_FRSTATE_ECU_INFO, p.frame_state);

    /* Feed 24 bytes: bytes 19-23 are the part number suffix */
    for (uint8_t i = 0; i < 19; i++) {
        ev = consult_parser_feed(&p, 0x00);
        TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    }
    /* Part number suffix: "6HK10" */
    consult_parser_feed(&p, '6');
    consult_parser_feed(&p, 'H');
    consult_parser_feed(&p, 'K');
    consult_parser_feed(&p, '1');
    ev = consult_parser_feed(&p, '0');
    TEST_ASSERT_EQUAL(CONSULT_EVENT_ECU_INFO_READY, ev);
    TEST_ASSERT_EQUAL_STRING("23710-6HK10", p.ecu_part_no);
}

/* --- Fault Codes --- */

void test_fault_codes_basic(void) {
    consult_parser_set_state(&p, CONSULT_STATE_WAITING_ECU_RESPONSE);

    /* ECU responds with inverted SELF_DIAG: ~0xD1 = 0x2E */
    consult_parser_feed(&p, 0x2E);
    TEST_ASSERT_TRUE(p.reading_frame);
    TEST_ASSERT_EQUAL(CONSULT_FRSTATE_FAULT_CODES, p.frame_state);

    /* Count byte: prev=0xFF (from reset), current=3 (3 fault codes) */
    consult_parser_feed(&p, 0xFF);
    consult_parser_feed(&p, 0x03);
    TEST_ASSERT_EQUAL_UINT8(3, p.fault_codes_count);

    /* Feed 3 fault code bytes */
    consult_parser_feed(&p, 0x11);
    consult_parser_feed(&p, 0x22);
    consult_event_t ev = consult_parser_feed(&p, 0x33);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_FAULT_CODES_READY, ev);

    TEST_ASSERT_EQUAL_HEX8(0x11, p.fault_codes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, p.fault_codes[1]);
    TEST_ASSERT_EQUAL_HEX8(0x33, p.fault_codes[2]);
}

void test_fault_codes_bounds_check(void) {
    consult_parser_set_state(&p, CONSULT_STATE_WAITING_ECU_RESPONSE);

    consult_parser_feed(&p, 0x2E);  /* ~0xD1 */

    /* Count byte: 200 > CONSULT_FAULT_CODES_BUF_SIZE (64) */
    consult_parser_feed(&p, 0xFF);
    consult_parser_feed(&p, 200);
    TEST_ASSERT_EQUAL_UINT8(CONSULT_FAULT_CODES_BUF_SIZE, p.fault_codes_count);
}

void test_fault_codes_zero_count(void) {
    consult_parser_set_state(&p, CONSULT_STATE_WAITING_ECU_RESPONSE);

    consult_parser_feed(&p, 0x2E);  /* ~0xD1 */

    /* Count = 0 — immediately completes with no fault codes */
    consult_parser_feed(&p, 0xFF);
    consult_event_t ev = consult_parser_feed(&p, 0x00);
    TEST_ASSERT_EQUAL_UINT8(0, p.fault_codes_count);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_FAULT_CODES_READY, ev);
    TEST_ASSERT_FALSE(p.reading_frame);
}

/* --- Clear Codes --- */

void test_clear_codes_ack(void) {
    consult_parser_set_state(&p, CONSULT_STATE_WAITING_ECU_RESPONSE);

    /* ~0xC1 = 0x3E */
    consult_event_t ev = consult_parser_feed(&p, 0x3E);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_CLEAR_CODES_OK, ev);
}

/* --- State management --- */

void test_set_state(void) {
    consult_parser_set_state(&p, CONSULT_STATE_STREAMING);
    TEST_ASSERT_EQUAL(CONSULT_STATE_STREAMING, p.state);
}

void test_bytes_ignored_in_wrong_state(void) {
    /* In STARTUP state, init response byte should be ignored */
    consult_event_t ev = consult_parser_feed(&p, 0x10);
    TEST_ASSERT_EQUAL(CONSULT_EVENT_NONE, ev);
    TEST_ASSERT_EQUAL(CONSULT_STATE_STARTUP, p.state);
}

void test_stream_sync_ignored_when_not_streaming(void) {
    consult_parser_set_state(&p, CONSULT_STATE_IDLE);
    consult_parser_feed(&p, 0xFF);
    consult_parser_feed(&p, CONSULT_STREAM_FRAME_SIZE);
    TEST_ASSERT_FALSE(p.reading_frame);
}

/* --- Plausibility check --- */

/* Layout indices (must mirror consult_parser.c internals). */
#define OFF_BATTERY   0
#define OFF_SPEED     8
#define OFF_TACH_MSB  9
#define OFF_TACH_LSB  10
#define OFF_INJ_MSB   11
#define OFF_INJ_LSB   12
#define OFF_MAF_MSB   13
#define OFF_MAF_LSB   14

static void fill_plausible(consult_parser_t *pp) {
    for (int i = 0; i < CONSULT_STREAM_FRAME_SIZE; i++) pp->data[i] = 0;
    pp->data[OFF_BATTERY]  = 175; /* 14.0 V */
    pp->data[OFF_SPEED]    = 30;  /* 60 km/h */
    pp->data[OFF_TACH_MSB] = 0x01;
    pp->data[OFF_TACH_LSB] = 0x40; /* raw 320 → 4000 rpm */
    pp->data[OFF_INJ_MSB]  = 0x01;
    pp->data[OFF_INJ_LSB]  = 0xF4; /* raw 500 → 5.00 ms */
    pp->data[OFF_MAF_MSB]  = 0x01;
    pp->data[OFF_MAF_LSB]  = 0x90; /* raw 400 → 2000 mV */
}

void test_validate_accepts_plausible(void) {
    fill_plausible(&p);
    TEST_ASSERT_TRUE(consult_parser_validate_stream(&p));
    TEST_ASSERT_TRUE(p.have_last_good);
    TEST_ASSERT_EQUAL(175, p.last_good_data[OFF_BATTERY]);
}

void test_validate_rejects_rpm_spike(void) {
    fill_plausible(&p);
    consult_parser_validate_stream(&p); /* seed last_good */

    /* Shifted frame: RPM MSB = 0xE9 → 745663 rpm */
    p.data[OFF_TACH_MSB] = 0xE9;
    p.data[OFF_TACH_LSB] = 0xC5;

    TEST_ASSERT_FALSE(consult_parser_validate_stream(&p));
    /* data[] rolled back to previous good snapshot */
    TEST_ASSERT_EQUAL(0x01, p.data[OFF_TACH_MSB]);
    TEST_ASSERT_EQUAL(0x40, p.data[OFF_TACH_LSB]);
}

void test_validate_rejects_impossible_speed(void) {
    fill_plausible(&p);
    consult_parser_validate_stream(&p);
    p.data[OFF_SPEED] = 200; /* 400 km/h */
    TEST_ASSERT_FALSE(consult_parser_validate_stream(&p));
}

void test_validate_rejects_low_battery(void) {
    fill_plausible(&p);
    consult_parser_validate_stream(&p);
    p.data[OFF_BATTERY] = 20; /* 1.6 V — wiring fault */
    TEST_ASSERT_FALSE(consult_parser_validate_stream(&p));
}

void test_validate_rejects_maf_over_5v(void) {
    fill_plausible(&p);
    consult_parser_validate_stream(&p);
    p.data[OFF_MAF_MSB] = 0x04; /* raw 0x0400 = 1024 * 5 = 5120 mV */
    p.data[OFF_MAF_LSB] = 0x00;
    TEST_ASSERT_FALSE(consult_parser_validate_stream(&p));
}

void test_validate_first_bad_frame_keeps_zeros(void) {
    /* No prior good frame — rollback has nothing, data[] stays as-is. */
    for (int i = 0; i < CONSULT_STREAM_FRAME_SIZE; i++) p.data[i] = 0xEE;
    TEST_ASSERT_FALSE(consult_parser_validate_stream(&p));
    TEST_ASSERT_FALSE(p.have_last_good);
}

void test_validate_zero_rpm_is_plausible(void) {
    /* Engine off — all zeros except battery in plausible range. */
    fill_plausible(&p);
    p.data[OFF_TACH_MSB] = 0;
    p.data[OFF_TACH_LSB] = 0;
    p.data[OFF_SPEED]    = 0;
    TEST_ASSERT_TRUE(consult_parser_validate_stream(&p));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_defaults);
    RUN_TEST(test_init_response);
    RUN_TEST(test_init_wrong_byte_ignored);
    RUN_TEST(test_stream_frame_complete);
    RUN_TEST(test_stream_sync_requires_0xff_prefix);
    RUN_TEST(test_stream_0xff_in_data_no_false_sync);
    RUN_TEST(test_ecu_info_parses_part_number);
    RUN_TEST(test_fault_codes_basic);
    RUN_TEST(test_fault_codes_bounds_check);
    RUN_TEST(test_fault_codes_zero_count);
    RUN_TEST(test_clear_codes_ack);
    RUN_TEST(test_set_state);
    RUN_TEST(test_bytes_ignored_in_wrong_state);
    RUN_TEST(test_stream_sync_ignored_when_not_streaming);
    RUN_TEST(test_validate_accepts_plausible);
    RUN_TEST(test_validate_rejects_rpm_spike);
    RUN_TEST(test_validate_rejects_impossible_speed);
    RUN_TEST(test_validate_rejects_low_battery);
    RUN_TEST(test_validate_rejects_maf_over_5v);
    RUN_TEST(test_validate_first_bad_frame_keeps_zeros);
    RUN_TEST(test_validate_zero_rpm_is_plausible);
    return UNITY_END();
}
