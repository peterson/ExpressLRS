/**
 * TDD test cases for compressed MAVLink codec.
 *
 * These tests define the expected behaviour BEFORE implementation.
 * Run with: pio test -e native -f test_compressed_mavlink
 *
 * Test strategy:
 * 1. Known-input → expected-output for encoder (hand-computed)
 * 2. Roundtrip: encode → decode → compare with original
 * 3. Delta encoding: verify field-level deltas for each message type
 * 4. int8 overflow → int16 promotion
 * 5. Keyframe forcing on interval expiry
 * 6. Scheduler: priority ordering and rate limiting
 */

#include <cstdint>
#include <cstring>
#include <cmath>
#include <unity.h>

// These headers don't exist yet — the tests will fail to compile until
// the codec is implemented. That's the point of TDD.
#include "CompressedMavlinkDefs.h"
#include "CompressedMavlinkEncoder.h"
#include "CompressedMavlinkDecoder.h"

// ---------------------------------------------------------------------------
// Constants from the implementation plan (must match CompressedMavlinkDefs.h)
// ---------------------------------------------------------------------------

// CRSF sub-type for compressed MAVLink
static const uint8_t COMPRESSED_MAVLINK_SUBTYPE = 0x01;

// Frame flag bits
static const uint8_t FRAME_FLAG_KEYFRAME    = 0x80;
static const uint8_t FRAME_FLAG_HAS_PROMOTE = 0x40;

// Stream indices (from default subscription table)
static const uint8_t STREAM_HEARTBEAT       = 0;
static const uint8_t STREAM_ATTITUDE        = 1;
static const uint8_t STREAM_POSITION        = 2;
static const uint8_t STREAM_SYS_STATUS      = 3;

// MAVLink message IDs
static const uint32_t MAVLINK_MSG_HEARTBEAT            = 0;
static const uint32_t MAVLINK_MSG_SYS_STATUS           = 1;
static const uint32_t MAVLINK_MSG_ATTITUDE             = 30;
static const uint32_t MAVLINK_MSG_GLOBAL_POSITION_INT  = 33;
static const uint32_t MAVLINK_MSG_COMMAND_ACK          = 77;

// Scale factors (from implementation plan)
static const float ATTITUDE_ROLL_SCALE    = 0.01f;   // rad per int8 unit
static const float ATTITUDE_PITCH_SCALE   = 0.01f;
static const float ATTITUDE_YAW_SCALE     = 0.02f;
static const int16_t POSITION_LAT_SCALE   = 30;      // degE7 per int8 unit
static const int16_t POSITION_LON_SCALE   = 30;
static const int16_t POSITION_ALT_SCALE   = 500;     // mm per int8 unit
static const int16_t POSITION_VEL_SCALE   = 5;       // cm/s per int8 unit
static const int16_t POSITION_HDG_SCALE   = 50;      // cdeg per int8 unit

// ---------------------------------------------------------------------------
// Helper: build a raw MAVLink ATTITUDE payload (28 bytes, little-endian)
// ---------------------------------------------------------------------------
static void build_attitude_payload(uint8_t *buf,
    uint32_t time_boot_ms, float roll, float pitch, float yaw,
    float rollspeed, float pitchspeed, float yawspeed)
{
    memcpy(buf + 0,  &time_boot_ms, 4);
    memcpy(buf + 4,  &roll, 4);
    memcpy(buf + 8,  &pitch, 4);
    memcpy(buf + 12, &yaw, 4);
    memcpy(buf + 16, &rollspeed, 4);
    memcpy(buf + 20, &pitchspeed, 4);
    memcpy(buf + 24, &yawspeed, 4);
}

// ---------------------------------------------------------------------------
// Helper: build a raw MAVLink GLOBAL_POSITION_INT payload (28 bytes, LE)
// ---------------------------------------------------------------------------
static void build_position_payload(uint8_t *buf,
    uint32_t time_boot_ms, int32_t lat, int32_t lon,
    int32_t alt, int32_t relative_alt,
    int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
    memcpy(buf + 0,  &time_boot_ms, 4);
    memcpy(buf + 4,  &lat, 4);
    memcpy(buf + 8,  &lon, 4);
    memcpy(buf + 12, &alt, 4);
    memcpy(buf + 16, &relative_alt, 4);
    memcpy(buf + 20, &vx, 2);
    memcpy(buf + 22, &vy, 2);
    memcpy(buf + 24, &vz, 2);
    memcpy(buf + 26, &hdg, 2);
}

// ---------------------------------------------------------------------------
// Helper: build a raw MAVLink HEARTBEAT payload (9 bytes, LE)
// ---------------------------------------------------------------------------
static void build_heartbeat_payload(uint8_t *buf,
    uint32_t custom_mode, uint8_t type, uint8_t autopilot,
    uint8_t base_mode, uint8_t system_status, uint8_t mavlink_version)
{
    memcpy(buf + 0, &custom_mode, 4);
    buf[4] = type;
    buf[5] = autopilot;
    buf[6] = base_mode;
    buf[7] = system_status;
    buf[8] = mavlink_version;
}

// ===========================================================================
// TEST: Encoder initialisation
// ===========================================================================

void test_encoder_init(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();
    // After init, encoder should have the default stream table loaded.
    // First encode of any supported message should produce a keyframe.
    // (Verified in subsequent tests.)
    TEST_PASS();
}

// ===========================================================================
// TEST: HEARTBEAT is always encoded as a keyframe
// ===========================================================================

void test_heartbeat_always_keyframe(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[9];
    build_heartbeat_payload(payload, 10 /* AUTO */, 1 /* FIXED_WING */,
                            3 /* ARDUPILOT */, 129 /* ARMED */, 4 /* ACTIVE */, 3);

    uint8_t out[64];
    uint8_t out_len = 0;

    bool produced = encoder.encode(MAVLINK_MSG_HEARTBEAT, payload, 9,
                                   out, &out_len, 1000);

    TEST_ASSERT_TRUE(produced);
    TEST_ASSERT_GREATER_THAN(0, out_len);

    // First byte = stream index
    TEST_ASSERT_EQUAL(STREAM_HEARTBEAT, out[0]);

    // Second byte = frame flags with keyframe bit set
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);

    // Payload should contain the raw heartbeat fields (9 bytes)
    TEST_ASSERT_EQUAL(2 + 9, out_len);  // index + flags + payload
}

// ===========================================================================
// TEST: First ATTITUDE encode produces a keyframe
// ===========================================================================

void test_attitude_first_encode_is_keyframe(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[28];
    build_attitude_payload(payload, 1000, 0.1f, -0.05f, 1.5f, 0.01f, 0.005f, 0.002f);

    uint8_t out[64];
    uint8_t out_len = 0;

    bool produced = encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28,
                                   out, &out_len, 1000);

    TEST_ASSERT_TRUE(produced);
    TEST_ASSERT_EQUAL(STREAM_ATTITUDE, out[0]);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);

    // Keyframe body = raw field values minus time_boot_ms (24 bytes: 6 floats)
    TEST_ASSERT_EQUAL(2 + 24, out_len);
}

// ===========================================================================
// TEST: Second ATTITUDE encode with small change produces a delta
// ===========================================================================

void test_attitude_delta_small_change(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload1[28];
    build_attitude_payload(payload1, 1000, 0.1f, -0.05f, 1.5f, 0.01f, 0.005f, 0.002f);

    uint8_t out[64];
    uint8_t out_len = 0;

    // First encode — keyframe
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload1, 28, out, &out_len, 1000);

    // Second encode — small roll change (0.05 rad = 5 units at 0.01 scale)
    uint8_t payload2[28];
    build_attitude_payload(payload2, 1100, 0.15f, -0.05f, 1.5f, 0.01f, 0.005f, 0.002f);

    out_len = 0;
    bool produced = encoder.encode(MAVLINK_MSG_ATTITUDE, payload2, 28,
                                   out, &out_len, 1100);

    TEST_ASSERT_TRUE(produced);
    TEST_ASSERT_EQUAL(STREAM_ATTITUDE, out[0]);

    // Should be a delta (keyframe bit NOT set)
    TEST_ASSERT_BITS_LOW(FRAME_FLAG_KEYFRAME, out[1]);

    // Bitmask: only roll changed (bit 0)
    TEST_ASSERT_EQUAL(0x01, out[2]);

    // No promotion needed
    TEST_ASSERT_BITS_LOW(FRAME_FLAG_HAS_PROMOTE, out[1]);

    // Delta value: (0.15 - 0.1) / 0.01 = 5 as int8
    TEST_ASSERT_EQUAL(5, (int8_t)out[3]);

    // Total: index(1) + flags(1) + bitmask(1) + delta(1) = 4 bytes
    TEST_ASSERT_EQUAL(4, out_len);
}

// ===========================================================================
// TEST: ATTITUDE delta with multiple field changes
// ===========================================================================

void test_attitude_delta_multiple_fields(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload1[28];
    build_attitude_payload(payload1, 1000, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    uint8_t out[64];
    uint8_t out_len = 0;

    // Keyframe
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload1, 28, out, &out_len, 1000);

    // Delta: roll +0.03 (3 units), pitch -0.02 (-2 units), yaw +0.1 (5 units at 0.02 scale)
    uint8_t payload2[28];
    build_attitude_payload(payload2, 1100, 0.03f, -0.02f, 0.1f, 0.0f, 0.0f, 0.0f);

    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload2, 28, out, &out_len, 1100);

    TEST_ASSERT_BITS_LOW(FRAME_FLAG_KEYFRAME, out[1]);

    // Bitmask: roll(0), pitch(1), yaw(2) = 0b00000111 = 0x07
    TEST_ASSERT_EQUAL(0x07, out[2]);

    // Deltas: roll=3, pitch=-2, yaw=5
    TEST_ASSERT_EQUAL(3, (int8_t)out[3]);
    TEST_ASSERT_EQUAL(-2, (int8_t)out[4]);
    TEST_ASSERT_EQUAL(5, (int8_t)out[5]);

    // Total: index(1) + flags(1) + bitmask(1) + 3 deltas(3) = 6 bytes
    TEST_ASSERT_EQUAL(6, out_len);
}

// ===========================================================================
// TEST: Delta with int16 promotion (large change exceeds int8 range)
// ===========================================================================

void test_attitude_delta_int16_promotion(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload1[28];
    build_attitude_payload(payload1, 1000, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    uint8_t out[64];
    uint8_t out_len = 0;

    encoder.encode(MAVLINK_MSG_ATTITUDE, payload1, 28, out, &out_len, 1000);

    // Large roll change: 2.0 rad = 200 units at 0.01 scale → exceeds int8 (±127)
    uint8_t payload2[28];
    build_attitude_payload(payload2, 1100, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload2, 28, out, &out_len, 1100);

    // Should have promotion flag set
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_HAS_PROMOTE, out[1]);

    // Bitmask: only roll
    TEST_ASSERT_EQUAL(0x01, out[2]);

    // Promotion bits: roll promoted (bit 0)
    TEST_ASSERT_EQUAL(0x01, out[3]);

    // Delta as int16 little-endian: 200
    int16_t delta;
    memcpy(&delta, out + 4, 2);
    TEST_ASSERT_EQUAL(200, delta);

    // Total: index(1) + flags(1) + bitmask(1) + promote(1) + int16(2) = 6 bytes
    TEST_ASSERT_EQUAL(6, out_len);
}

// ===========================================================================
// TEST: GLOBAL_POSITION_INT delta encoding
// ===========================================================================

void test_position_delta(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    // Position near Canberra: -35.2809°, 149.1300°
    uint8_t payload1[28];
    build_position_payload(payload1, 1000,
        -352809000, 1491300000,   // lat, lon (degE7)
        905000, 610000,            // alt, rel_alt (mm)
        2200, -500, 0,             // vx, vy, vz (cm/s)
        24380);                    // hdg (cdeg = 243.8°)

    uint8_t out[64];
    uint8_t out_len = 0;

    // Keyframe
    encoder.encode(MAVLINK_MSG_GLOBAL_POSITION_INT, payload1, 28,
                   out, &out_len, 1000);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);

    // Small position change: lat +90 degE7 (~10m), lon -60 degE7 (~5m)
    // At scale 30: lat delta = 90/30 = 3 units, lon delta = -60/30 = -2 units
    uint8_t payload2[28];
    build_position_payload(payload2, 1200,
        -352809000 + 90, 1491300000 - 60,
        905000, 610000,
        2200, -500, 0,
        24380);

    out_len = 0;
    encoder.encode(MAVLINK_MSG_GLOBAL_POSITION_INT, payload2, 28,
                   out, &out_len, 1200);

    TEST_ASSERT_BITS_LOW(FRAME_FLAG_KEYFRAME, out[1]);

    // Only lat and lon changed: bitmask bits 0,1 = 0x03
    TEST_ASSERT_EQUAL(0x03, out[2]);

    // Deltas: lat=3, lon=-2
    TEST_ASSERT_EQUAL(3, (int8_t)out[3]);
    TEST_ASSERT_EQUAL(-2, (int8_t)out[4]);

    // Total: 5 bytes
    TEST_ASSERT_EQUAL(5, out_len);
}

// ===========================================================================
// TEST: No-change frame produces no output (all deltas zero)
// ===========================================================================

void test_no_change_produces_no_output(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[28];
    build_attitude_payload(payload, 1000, 0.1f, -0.05f, 1.5f, 0.01f, 0.005f, 0.002f);

    uint8_t out[64];
    uint8_t out_len = 0;

    // Keyframe
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1000);

    // Same values, only time_boot_ms changed (which is excluded)
    build_attitude_payload(payload, 1100, 0.1f, -0.05f, 1.5f, 0.01f, 0.005f, 0.002f);

    out_len = 0;
    bool produced = encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28,
                                   out, &out_len, 1100);

    // No fields changed → no frame produced (saves bandwidth)
    TEST_ASSERT_FALSE(produced);
    TEST_ASSERT_EQUAL(0, out_len);
}

// ===========================================================================
// TEST: Keyframe forced after interval expires
// ===========================================================================

void test_keyframe_forced_on_interval(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[28];
    build_attitude_payload(payload, 1000, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    uint8_t out[64];
    uint8_t out_len = 0;

    // First encode at t=1000 — keyframe
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1000);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);

    // Encode at t=1100 (100ms later) — should be delta
    build_attitude_payload(payload, 1100, 0.11f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1100);
    TEST_ASSERT_BITS_LOW(FRAME_FLAG_KEYFRAME, out[1]);

    // Encode at t=3100 (2100ms after first keyframe, >2000ms interval) — forced keyframe
    build_attitude_payload(payload, 3100, 0.12f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 3100);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);
}

// ===========================================================================
// TEST: Roundtrip — encode then decode, compare with original
// ===========================================================================

void test_roundtrip_attitude_keyframe(void)
{
    CompressedMavlinkEncoder encoder;
    CompressedMavlinkDecoder decoder;
    encoder.init();
    decoder.init();

    uint8_t payload[28];
    float roll = 0.1234f, pitch = -0.0567f, yaw = 2.345f;
    float rs = 0.012f, ps = -0.003f, ys = 0.001f;
    build_attitude_payload(payload, 5000, roll, pitch, yaw, rs, ps, ys);

    uint8_t compressed[64];
    uint8_t compressed_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28,
                   compressed, &compressed_len, 5000);

    uint8_t decoded[64];
    uint8_t decoded_len = 0;
    bool ok = decoder.decode(compressed, compressed_len, decoded, &decoded_len);

    TEST_ASSERT_TRUE(ok);

    // Decoded payload should match original field values
    // (time_boot_ms may differ since it's excluded from compression)
    float decoded_roll, decoded_pitch, decoded_yaw;
    memcpy(&decoded_roll,  decoded + 4, 4);
    memcpy(&decoded_pitch, decoded + 8, 4);
    memcpy(&decoded_yaw,   decoded + 12, 4);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, roll, decoded_roll);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, pitch, decoded_pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, yaw, decoded_yaw);
}

void test_roundtrip_attitude_delta(void)
{
    CompressedMavlinkEncoder encoder;
    CompressedMavlinkDecoder decoder;
    encoder.init();
    decoder.init();

    // Keyframe
    uint8_t payload1[28];
    build_attitude_payload(payload1, 1000, 0.1f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    uint8_t compressed[64];
    uint8_t cl = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload1, 28, compressed, &cl, 1000);

    uint8_t decoded[64];
    uint8_t dl = 0;
    decoder.decode(compressed, cl, decoded, &dl);

    // Delta: roll changes by +0.05 (5 units)
    uint8_t payload2[28];
    build_attitude_payload(payload2, 1100, 0.15f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    cl = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload2, 28, compressed, &cl, 1100);

    dl = 0;
    bool ok = decoder.decode(compressed, cl, decoded, &dl);
    TEST_ASSERT_TRUE(ok);

    float decoded_roll;
    memcpy(&decoded_roll, decoded + 4, 4);

    // Quantisation: 0.15 → round to nearest 0.01 = 0.15 (exact at this scale)
    // But the delta is computed as integer units, so:
    // original 0.1 → encoded as keyframe (exact IEEE 754)
    // delta = (0.15 - 0.1) / 0.01 = 5 → decoded as 0.1 + 5 * 0.01 = 0.15
    TEST_ASSERT_FLOAT_WITHIN(ATTITUDE_ROLL_SCALE, 0.15f, decoded_roll);
}

void test_roundtrip_position(void)
{
    CompressedMavlinkEncoder encoder;
    CompressedMavlinkDecoder decoder;
    encoder.init();
    decoder.init();

    uint8_t payload1[28];
    int32_t lat1 = -352809000, lon1 = 1491300000;
    int32_t alt1 = 905000, ralt1 = 610000;
    build_position_payload(payload1, 1000, lat1, lon1, alt1, ralt1, 2200, -500, 0, 24380);

    uint8_t compressed[64];
    uint8_t cl = 0;
    encoder.encode(MAVLINK_MSG_GLOBAL_POSITION_INT, payload1, 28, compressed, &cl, 1000);

    uint8_t decoded[64];
    uint8_t dl = 0;
    decoder.decode(compressed, cl, decoded, &dl);

    // Delta: lat +90, lon -60
    uint8_t payload2[28];
    build_position_payload(payload2, 1200, lat1 + 90, lon1 - 60, alt1, ralt1, 2200, -500, 0, 24380);

    cl = 0;
    encoder.encode(MAVLINK_MSG_GLOBAL_POSITION_INT, payload2, 28, compressed, &cl, 1200);

    dl = 0;
    bool ok = decoder.decode(compressed, cl, decoded, &dl);
    TEST_ASSERT_TRUE(ok);

    int32_t decoded_lat, decoded_lon;
    memcpy(&decoded_lat, decoded + 4, 4);
    memcpy(&decoded_lon, decoded + 8, 4);

    // Quantisation: 90 / 30 = 3 units → decoded as lat1 + 3*30 = lat1 + 90 (exact)
    TEST_ASSERT_INT32_WITHIN(POSITION_LAT_SCALE, lat1 + 90, decoded_lat);
    TEST_ASSERT_INT32_WITHIN(POSITION_LON_SCALE, lon1 - 60, decoded_lon);
}

// ===========================================================================
// TEST: Unknown message ID is ignored
// ===========================================================================

void test_unknown_msgid_ignored(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[10] = {0};
    uint8_t out[64];
    uint8_t out_len = 0;

    bool produced = encoder.encode(999, payload, 10, out, &out_len, 1000);
    TEST_ASSERT_FALSE(produced);
    TEST_ASSERT_EQUAL(0, out_len);
}

// ===========================================================================
// TEST: Decoder handles corrupted stream index gracefully
// ===========================================================================

void test_decoder_invalid_stream_index(void)
{
    CompressedMavlinkDecoder decoder;
    decoder.init();

    uint8_t bad_frame[] = {254, 0x80, 0x00};  // stream 254 doesn't exist
    uint8_t decoded[64];
    uint8_t dl = 0;

    bool ok = decoder.decode(bad_frame, 3, decoded, &dl);
    TEST_ASSERT_FALSE(ok);
}

// ===========================================================================
// TEST: Reset forces keyframes on all streams
// ===========================================================================

void test_reset_forces_keyframes(void)
{
    CompressedMavlinkEncoder encoder;
    encoder.init();

    uint8_t payload[28];
    build_attitude_payload(payload, 1000, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    uint8_t out[64];
    uint8_t out_len = 0;

    // Keyframe
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1000);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);

    // Delta
    build_attitude_payload(payload, 1100, 0.11f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1100);
    TEST_ASSERT_BITS_LOW(FRAME_FLAG_KEYFRAME, out[1]);

    // Reset
    encoder.resetAllStreams();

    // Next encode should be a keyframe again
    build_attitude_payload(payload, 1200, 0.12f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    out_len = 0;
    encoder.encode(MAVLINK_MSG_ATTITUDE, payload, 28, out, &out_len, 1200);
    TEST_ASSERT_BITS_HIGH(FRAME_FLAG_KEYFRAME, out[1]);
}

// ===========================================================================
// Entry point
// ===========================================================================

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Encoder basics
    RUN_TEST(test_encoder_init);
    RUN_TEST(test_heartbeat_always_keyframe);
    RUN_TEST(test_attitude_first_encode_is_keyframe);
    RUN_TEST(test_unknown_msgid_ignored);

    // Delta encoding
    RUN_TEST(test_attitude_delta_small_change);
    RUN_TEST(test_attitude_delta_multiple_fields);
    RUN_TEST(test_attitude_delta_int16_promotion);
    RUN_TEST(test_position_delta);
    RUN_TEST(test_no_change_produces_no_output);

    // Keyframe management
    RUN_TEST(test_keyframe_forced_on_interval);
    RUN_TEST(test_reset_forces_keyframes);

    // Roundtrip (encode → decode)
    RUN_TEST(test_roundtrip_attitude_keyframe);
    RUN_TEST(test_roundtrip_attitude_delta);
    RUN_TEST(test_roundtrip_position);

    // Error handling
    RUN_TEST(test_decoder_invalid_stream_index);

    return UNITY_END();
}
