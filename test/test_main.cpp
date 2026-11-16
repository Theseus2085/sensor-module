/**
 * test_main.cpp - Unit tests for the REAL sensor-module logic.
 *
 * Compiles src/main.cpp natively against test/stubs/mbed.h (PlatformIO
 * native env) and exercises the measurement/payload chain end to end:
 *   ADC -> calibration interpolation -> x10000 -> digits -> version+CRC frame
 */

#include <unity.h>
#include <cstdint>
#include <cstring>

// ---- real firmware globals/functions under test (from src/main.cpp) ----
struct CalibrationPoint {
    uint16_t raw_adc;
    float diameter_mm;
};
extern CalibrationPoint calibration_tables[2][3];
extern volatile float sensor1_mm, sensor2_mm;
extern volatile uint8_t tx_buffer[12];

extern uint8_t crc8(const uint8_t *data, uint8_t length);
extern uint32_t mm_to_fixed_10000(float val);
extern void format_sensor_data_fixed(uint32_t val_x10000, uint8_t *buf);
extern void build_payload(float s1_mm, float s2_mm, uint8_t *buf);
extern float convert_raw_adc_to_mm(uint16_t raw_adc, uint8_t sensor_idx);
extern void measure_sensor_values(void);
extern void reinit_i2c_slave(void);

// default rig table (mirrors firmware defaults)
static const uint16_t D_RAW[3] = { 7, 532, 1119 };
static const float   D_MM[3]   = { 1.47f, 1.68f, 1.99f };

static uint32_t decode5(const uint8_t *b) {
    uint32_t v = 0;
    for (int i = 0; i < 5; i++) v = v * 10u + b[i];
    return v;
}

void setUp() {
    // restore the default rig calibration before each test
    for (int s = 0; s < 2; s++)
        for (int p = 0; p < 3; p++) {
            calibration_tables[s][p].raw_adc = D_RAW[p];
            calibration_tables[s][p].diameter_mm = D_MM[p];
        }
}
void tearDown() {}

// =========================== CRC8 ===========================

void test_crc8_reference_vectors(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, crc8((const uint8_t *)"", 0));
    TEST_ASSERT_EQUAL_UINT8(0x00, crc8((const uint8_t *)"\x00", 1));
    TEST_ASSERT_EQUAL_UINT8(0xF3, crc8((const uint8_t *)"\xFF", 1));
    TEST_ASSERT_EQUAL_UINT8(0x58, crc8((const uint8_t *)"\xAB", 1));
    TEST_ASSERT_EQUAL_UINT8(0xF4, crc8((const uint8_t *)"123456789", 9)); // SMBus check value
}

void test_crc8_multi_byte(void) {
    const uint8_t f[11] = { 1,7,5,0,0, 1,7,5,0,0, 0x01 };
    TEST_ASSERT_EQUAL_UINT8(0xD3, crc8(f, 11));
    const uint8_t g[11] = { 1,9,9,9,9, 1,4,7,0,0, 0x01 };
    TEST_ASSERT_EQUAL_UINT8(0xFB, crc8(g, 11));
}

// ===================== mm -> x10000 =====================

void test_mm_to_fixed_basic(void) {
    TEST_ASSERT_EQUAL_UINT32(17500, mm_to_fixed_10000(1.75f));
    TEST_ASSERT_EQUAL_UINT32(14700, mm_to_fixed_10000(1.47f));
    TEST_ASSERT_EQUAL_UINT32(20000, mm_to_fixed_10000(2.0f));
    TEST_ASSERT_EQUAL_UINT32(0, mm_to_fixed_10000(0.0f));
}

void test_mm_to_fixed_clamps(void) {
    TEST_ASSERT_EQUAL_UINT32(0, mm_to_fixed_10000(-1.5f));       // negative -> 0
    TEST_ASSERT_EQUAL_UINT32(99999, mm_to_fixed_10000(10.0f));   // > 9.9999 -> clamp
    TEST_ASSERT_EQUAL_UINT32(99999, mm_to_fixed_10000(99.0f));
}

void test_mm_to_fixed_rounding(void) {
    // 1.75005 -> 17500.5 -> +0.5 -> 17501 (round-half-up)
    TEST_ASSERT_EQUAL_UINT32(17501, mm_to_fixed_10000(1.75005f));
    TEST_ASSERT_EQUAL_UINT32(17500, mm_to_fixed_10000(1.75004f));
}

// ===================== digit formatting =====================

void test_format_digits(void) {
    uint8_t buf[5];
    format_sensor_data_fixed(17500, buf);
    const uint8_t exp[5] = { 1,7,5,0,0 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(exp, buf, 5);
    format_sensor_data_fixed(0, buf);
    const uint8_t z[5] = { 0,0,0,0,0 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(z, buf, 5);
    format_sensor_data_fixed(99999, buf);
    const uint8_t n[5] = { 9,9,9,9,9 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(n, buf, 5);
    format_sensor_data_fixed(12345, buf);
    const uint8_t c[5] = { 1,2,3,4,5 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(c, buf, 5);
    format_sensor_data_fixed(100000, buf); // over max -> clamped
    TEST_ASSERT_EQUAL_UINT8_ARRAY(n, buf, 5);
}

// ===================== frame assembly =====================

void test_build_payload_layout(void) {
    uint8_t f[12];
    memset(f, 0xEE, sizeof f);
    build_payload(1.75f, 1.75f, f);
    TEST_ASSERT_EQUAL_UINT8(0x01, f[10]);              // version byte
    TEST_ASSERT_EQUAL_UINT8(crc8(f, 11), f[11]);       // CRC over bytes 0..10
    TEST_ASSERT_EQUAL_UINT8(1, f[0]); TEST_ASSERT_EQUAL_UINT8(7, f[1]);
    TEST_ASSERT_EQUAL_UINT8(5, f[2]); TEST_ASSERT_EQUAL_UINT8(0, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0, f[4]);
}

void test_build_payload_roundtrip(void) {
    const float vals[] = { 0.5f, 1.47f, 1.5f, 1.68f, 1.75f, 1.8f, 1.99f, 2.0f, 2.35f, 3.5f };
    uint8_t f[12];
    for (float v : vals) {
        build_payload(v, v, f);
        uint32_t dec = decode5(f);
        uint32_t exp = mm_to_fixed_10000(v);
        TEST_ASSERT_EQUAL_UINT32(exp, dec);
        TEST_ASSERT_EQUAL_UINT32(exp, decode5(f + 5));
        TEST_ASSERT_EQUAL_UINT8(0x01, f[10]);
        TEST_ASSERT_EQUAL_UINT8(crc8(f, 11), f[11]);
    }
}

// ===================== calibration interpolation =====================

void test_convert_exact_calibration_points(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.47f, convert_raw_adc_to_mm(D_RAW[0], 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.68f, convert_raw_adc_to_mm(D_RAW[1], 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.99f, convert_raw_adc_to_mm(D_RAW[2], 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.68f, convert_raw_adc_to_mm(D_RAW[1], 1));
}

void test_convert_interpolation_segments(void) {
    // segment 0: raw 7..532 maps 1.47..1.68
    float d269 = convert_raw_adc_to_mm(269, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.47f + (1.68f - 1.47f) * (269 - 7) / 525.0f, d269);
    TEST_ASSERT_TRUE(d269 > 1.47f && d269 < 1.68f);
    // segment 1: raw 532..1119 maps 1.68..1.99
    float d825 = convert_raw_adc_to_mm(825, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.68f + (1.99f - 1.68f) * (825 - 532) / 587.0f, d825);
    TEST_ASSERT_TRUE(d825 > 1.68f && d825 < 1.99f);
    // monotony across the whole envelope
    TEST_ASSERT_TRUE(convert_raw_adc_to_mm(100, 0) < convert_raw_adc_to_mm(400, 0));
    TEST_ASSERT_TRUE(convert_raw_adc_to_mm(600, 0) < convert_raw_adc_to_mm(1000, 0));
}

void test_convert_clamps_to_calibrated_envelope(void) {
    // below/above the calibrated range -> clamped, no extrapolation (FW 0.7.0)
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.47f, convert_raw_adc_to_mm(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.47f, convert_raw_adc_to_mm(6, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.99f, convert_raw_adc_to_mm(1120, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.99f, convert_raw_adc_to_mm(4000, 0));
}

void test_convert_branch_boundary_at_midpoint(void) {
    // raw == table[1] must use the first branch and hit the point exactly
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.68f, convert_raw_adc_to_mm(532, 0));
    // raw == 531 (just below) still first branch, close to 1.68
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.6796f, convert_raw_adc_to_mm(531, 0));
}

void test_convert_sensor_index_guard(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.75f, convert_raw_adc_to_mm(500, 2));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.75f, convert_raw_adc_to_mm(500, 255));
}

void test_convert_descending_table_stays_in_envelope(void) {
    // a hypothetically reversed capture order must not break the clamping
    calibration_tables[0][0] = { 1119, 1.99f };
    calibration_tables[0][1] = { 532, 1.68f };
    calibration_tables[0][2] = { 7, 1.47f };
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.47f, convert_raw_adc_to_mm(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.99f, convert_raw_adc_to_mm(4000, 0));
}

void test_convert_degenerate_equal_raws_no_div0(void) {
    for (int p = 0; p < 3; p++) calibration_tables[1][p] = { 100, 1.5f + 0.01f * p };
    float d = convert_raw_adc_to_mm(100, 1); // all denominators zero -> fallback
    TEST_ASSERT_TRUE(d >= 1.5f && d <= 1.52f);
}

// ===================== measurement chain =====================

void test_measure_sensor_values_end_to_end(void) {
    // stub defaults read 0.5 -> raw = (uint16)(0.5*4095) = 2047 per sample,
    // burst mean over 16 identical samples = 2047
    measure_sensor_values();
    float exp = convert_raw_adc_to_mm(2047, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, exp, sensor1_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, exp, sensor2_mm);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_reference_vectors);
    RUN_TEST(test_crc8_multi_byte);
    RUN_TEST(test_mm_to_fixed_basic);
    RUN_TEST(test_mm_to_fixed_clamps);
    RUN_TEST(test_mm_to_fixed_rounding);
    RUN_TEST(test_format_digits);
    RUN_TEST(test_build_payload_layout);
    RUN_TEST(test_build_payload_roundtrip);
    RUN_TEST(test_convert_exact_calibration_points);
    RUN_TEST(test_convert_interpolation_segments);
    RUN_TEST(test_convert_clamps_to_calibrated_envelope);
    RUN_TEST(test_convert_branch_boundary_at_midpoint);
    RUN_TEST(test_convert_sensor_index_guard);
    RUN_TEST(test_convert_descending_table_stays_in_envelope);
    RUN_TEST(test_convert_degenerate_equal_raws_no_div0);
    RUN_TEST(test_measure_sensor_values_end_to_end);
    return UNITY_END();
}
