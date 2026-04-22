#include "CompressedMavlinkDecoder.h"
#include <cstring>
#include <cmath>

// ---------------------------------------------------------------------------
// Default stream table — must match encoder's table exactly
// ---------------------------------------------------------------------------

void CompressedMavlinkDecoder::initDefaultStreams()
{
    m_num_streams = 0;

    #define ADD_STREAM(idx, msgid, prio, nf, payoff, rawsz, always_kf, kf_ms) \
        { \
            auto &s = m_streams[m_num_streams++]; \
            s.stream_index = idx; \
            s.mavlink_msgid = msgid; \
            s.priority = prio; \
            s.num_fields = nf; \
            s.payload_offset = payoff; \
            s.raw_payload_size = rawsz; \
            s.always_keyframe = always_kf; \
            s.keyframe_interval_ms = kf_ms; \
            s.last_keyframe_ms = 0; \
            s.has_keyframe = false; \
            memset(s.last_values, 0, sizeof(s.last_values)); \
            memset(s.field_sizes, 0, sizeof(s.field_sizes)); \
            memset(s.float_scales, 0, sizeof(s.float_scales)); \
            memset(s.int_scales, 0, sizeof(s.int_scales)); \
        }

    ADD_STREAM(CMAV_STREAM_HEARTBEAT, CMAV_MSGID_HEARTBEAT, 0, 0, 0, 9, true, 0);

    ADD_STREAM(CMAV_STREAM_ATTITUDE, CMAV_MSGID_ATTITUDE, 1, 6, 4, 28, false, CMAV_DEFAULT_KF_INTERVAL_MS);
    {
        auto &s = m_streams[CMAV_STREAM_ATTITUDE];
        for (uint8_t i = 0; i < 6; i++) s.field_sizes[i] = 4;
        s.float_scales[0] = CMAV_SCALE_ATT_ROLL;
        s.float_scales[1] = CMAV_SCALE_ATT_PITCH;
        s.float_scales[2] = CMAV_SCALE_ATT_YAW;
        s.float_scales[3] = CMAV_SCALE_ATT_ROLLSPEED;
        s.float_scales[4] = CMAV_SCALE_ATT_PITCHSPEED;
        s.float_scales[5] = CMAV_SCALE_ATT_YAWSPEED;
    }

    ADD_STREAM(CMAV_STREAM_POSITION, CMAV_MSGID_GLOBAL_POSITION_INT, 1, 8, 4, 28, false, CMAV_DEFAULT_KF_INTERVAL_MS);
    {
        auto &s = m_streams[CMAV_STREAM_POSITION];
        s.field_sizes[0] = 4; s.int_scales[0] = CMAV_SCALE_POS_LAT;
        s.field_sizes[1] = 4; s.int_scales[1] = CMAV_SCALE_POS_LON;
        s.field_sizes[2] = 4; s.int_scales[2] = CMAV_SCALE_POS_ALT;
        s.field_sizes[3] = 4; s.int_scales[3] = CMAV_SCALE_POS_RALT;
        s.field_sizes[4] = 2; s.int_scales[4] = CMAV_SCALE_POS_VX;
        s.field_sizes[5] = 2; s.int_scales[5] = CMAV_SCALE_POS_VY;
        s.field_sizes[6] = 2; s.int_scales[6] = CMAV_SCALE_POS_VZ;
        s.field_sizes[7] = 2; s.int_scales[7] = CMAV_SCALE_POS_HDG;
    }

    ADD_STREAM(CMAV_STREAM_SYS_STATUS, CMAV_MSGID_SYS_STATUS, 2, 4, 12, 31, false, CMAV_DEFAULT_KF_INTERVAL_MS);
    {
        auto &s = m_streams[CMAV_STREAM_SYS_STATUS];
        s.field_sizes[0] = 2; s.int_scales[0] = CMAV_SCALE_SYS_VOLTAGE;
        s.field_sizes[1] = 2; s.int_scales[1] = CMAV_SCALE_SYS_CURRENT;
        s.field_sizes[2] = 2; s.int_scales[2] = CMAV_SCALE_SYS_LOAD;
        s.field_sizes[3] = 1; s.int_scales[3] = CMAV_SCALE_SYS_REMAIN;
    }

    ADD_STREAM(CMAV_STREAM_STATUSTEXT, CMAV_MSGID_STATUSTEXT, 0, 0, 0, 51, true, 0);
    ADD_STREAM(CMAV_STREAM_CMD_ACK, CMAV_MSGID_COMMAND_ACK, 0, 0, 0, 3, true, 0);

    #undef ADD_STREAM
}

void CompressedMavlinkDecoder::init()
{
    initDefaultStreams();
}

cmav_stream_t *CompressedMavlinkDecoder::findStreamByIndex(uint8_t stream_index)
{
    for (uint8_t i = 0; i < m_num_streams; i++) {
        if (m_streams[i].stream_index == stream_index) return &m_streams[i];
    }
    return nullptr;
}

uint8_t CompressedMavlinkDecoder::requestKeyframe(uint8_t stream_index)
{
    auto *s = findStreamByIndex(stream_index);
    if (!s) return 0xFF;
    s->has_keyframe = false;
    return stream_index;
}

// ---------------------------------------------------------------------------
// Decode entry point
// ---------------------------------------------------------------------------

bool CompressedMavlinkDecoder::decode(const uint8_t *compressed, uint8_t compressed_len,
                                       uint8_t *mavlink_out, uint8_t *mavlink_len)
{
    *mavlink_len = 0;
    if (compressed_len < 2) return false;

    uint8_t stream_index = compressed[0];
    uint8_t flags = compressed[1];

    cmav_stream_t *stream = findStreamByIndex(stream_index);
    if (!stream) return false;

    const uint8_t *body = compressed + 2;
    uint8_t body_len = compressed_len - 2;

    if (flags & CMAV_FLAG_KEYFRAME) {
        return decodeKeyframe(*stream, body, body_len, mavlink_out, mavlink_len);
    } else {
        return decodeDelta(*stream, flags, body, body_len, mavlink_out, mavlink_len);
    }
}

// ---------------------------------------------------------------------------
// Keyframe decoding
// ---------------------------------------------------------------------------

bool CompressedMavlinkDecoder::decodeKeyframe(cmav_stream_t &stream,
                                                const uint8_t *body, uint8_t body_len,
                                                uint8_t *mavlink_out, uint8_t *mavlink_len)
{
    uint8_t copy_offset = stream.always_keyframe ? 0 : stream.payload_offset;
    uint8_t expected_len = stream.raw_payload_size - copy_offset;

    if (body_len < expected_len) return false;

    // Reconstruct full payload
    // For delta-encoded streams, zero the prefix (time_boot_ms etc) and copy fields
    if (!stream.always_keyframe) {
        memset(mavlink_out, 0, stream.payload_offset);
        memcpy(mavlink_out + stream.payload_offset, body, expected_len);
        // Store for future delta reconstruction
        memcpy(stream.last_values, body, expected_len);
    } else {
        memcpy(mavlink_out, body, expected_len);
    }

    *mavlink_len = stream.raw_payload_size;
    stream.has_keyframe = true;
    return true;
}

// ---------------------------------------------------------------------------
// Delta decoding
// ---------------------------------------------------------------------------

void CompressedMavlinkDecoder::applyIntDelta(uint8_t *field, uint8_t field_size,
                                               int32_t delta)
{
    int32_t val = 0;
    memcpy(&val, field, field_size);
    if (field_size == 2) val = (int16_t)val;
    else if (field_size == 1) val = (int8_t)val;
    val += delta;
    memcpy(field, &val, field_size);
}

void CompressedMavlinkDecoder::applyFloatDelta(uint8_t *field, float scale,
                                                 int32_t delta_units)
{
    float val;
    memcpy(&val, field, 4);
    val += delta_units * scale;
    memcpy(field, &val, 4);
}

bool CompressedMavlinkDecoder::decodeDelta(cmav_stream_t &stream, uint8_t flags,
                                             const uint8_t *body, uint8_t body_len,
                                             uint8_t *mavlink_out, uint8_t *mavlink_len)
{
    if (!stream.has_keyframe) return false;  // can't apply delta without baseline
    if (body_len < 1) return false;

    uint8_t pos = 0;
    uint8_t bitmask = body[pos++];

    uint8_t promote_bits = 0;
    if (flags & CMAV_FLAG_HAS_PROMOTE) {
        if (pos >= body_len) return false;
        promote_bits = body[pos++];
    }

    // Start from last known values
    uint8_t field_data_len = stream.raw_payload_size - stream.payload_offset;
    uint8_t working[CMAV_LAST_VALUES_BUF_SIZE];
    memcpy(working, stream.last_values, field_data_len);

    // Apply deltas
    uint8_t field_offset = 0;
    for (uint8_t i = 0; i < stream.num_fields; i++) {
        if (bitmask & (1 << i)) {
            int32_t delta;
            if (promote_bits & (1 << i)) {
                if (pos + 2 > body_len) return false;
                int16_t d16;
                memcpy(&d16, body + pos, 2);
                delta = d16;
                pos += 2;
            } else {
                if (pos >= body_len) return false;
                delta = (int8_t)body[pos++];
            }

            if (stream.float_scales[i] != 0.0f) {
                applyFloatDelta(working + field_offset, stream.float_scales[i], delta);
            } else {
                int32_t scaled_delta = delta * stream.int_scales[i];
                applyIntDelta(working + field_offset, stream.field_sizes[i], scaled_delta);
            }
        }
        field_offset += stream.field_sizes[i];
    }

    // Store updated values
    memcpy(stream.last_values, working, field_data_len);

    // Reconstruct full payload
    memset(mavlink_out, 0, stream.payload_offset);
    memcpy(mavlink_out + stream.payload_offset, working, field_data_len);
    *mavlink_len = stream.raw_payload_size;

    return true;
}
