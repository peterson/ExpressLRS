#include "CompressedMavlinkEncoder.h"
#include <cstring>
#include <cmath>

// ---------------------------------------------------------------------------
// Default stream table initialisation
// ---------------------------------------------------------------------------

void CompressedMavlinkEncoder::initDefaultStreams()
{
    m_num_streams = 0;

    // Helper macro to reduce repetition
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

    // Stream 0: HEARTBEAT — always keyframe, 9 bytes, no delta
    ADD_STREAM(CMAV_STREAM_HEARTBEAT, CMAV_MSGID_HEARTBEAT, 0, 0, 0, 9, true, 0);

    // Stream 1: ATTITUDE — 6 float fields after time_boot_ms (4 bytes)
    ADD_STREAM(CMAV_STREAM_ATTITUDE, CMAV_MSGID_ATTITUDE, 1, 6, 4, 28, false, CMAV_DEFAULT_KF_INTERVAL_MS);
    {
        auto &s = m_streams[CMAV_STREAM_ATTITUDE];
        for (uint8_t i = 0; i < 6; i++) s.field_sizes[i] = 4;  // all floats
        s.float_scales[0] = CMAV_SCALE_ATT_ROLL;
        s.float_scales[1] = CMAV_SCALE_ATT_PITCH;
        s.float_scales[2] = CMAV_SCALE_ATT_YAW;
        s.float_scales[3] = CMAV_SCALE_ATT_ROLLSPEED;
        s.float_scales[4] = CMAV_SCALE_ATT_PITCHSPEED;
        s.float_scales[5] = CMAV_SCALE_ATT_YAWSPEED;
    }

    // Stream 2: GLOBAL_POSITION_INT — 4×int32 + 3×int16 + 1×uint16 after time_boot_ms
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

    // Stream 3: SYS_STATUS — skip first 12 bytes (3×uint32 sensor fields), then 4 fields
    ADD_STREAM(CMAV_STREAM_SYS_STATUS, CMAV_MSGID_SYS_STATUS, 2, 4, 12, 31, false, CMAV_DEFAULT_KF_INTERVAL_MS);
    {
        auto &s = m_streams[CMAV_STREAM_SYS_STATUS];
        s.field_sizes[0] = 2; s.int_scales[0] = CMAV_SCALE_SYS_VOLTAGE;  // uint16 voltage
        s.field_sizes[1] = 2; s.int_scales[1] = CMAV_SCALE_SYS_CURRENT;  // int16 current
        s.field_sizes[2] = 2; s.int_scales[2] = CMAV_SCALE_SYS_LOAD;     // uint16 load
        s.field_sizes[3] = 1; s.int_scales[3] = CMAV_SCALE_SYS_REMAIN;   // int8 remaining
    }

    // Stream 6: STATUSTEXT — always keyframe
    ADD_STREAM(CMAV_STREAM_STATUSTEXT, CMAV_MSGID_STATUSTEXT, 0, 0, 0, 51, true, 0);

    // Stream 7: COMMAND_ACK — always keyframe
    ADD_STREAM(CMAV_STREAM_CMD_ACK, CMAV_MSGID_COMMAND_ACK, 0, 0, 0, 3, true, 0);

    #undef ADD_STREAM
}

void CompressedMavlinkEncoder::init()
{
    m_bandwidth = 625;  // default: 250 Hz OTA8 1:4
    initDefaultStreams();
}

void CompressedMavlinkEncoder::resetAllStreams()
{
    for (uint8_t i = 0; i < m_num_streams; i++) {
        m_streams[i].has_keyframe = false;
        m_streams[i].last_keyframe_ms = 0;
    }
}

void CompressedMavlinkEncoder::setBandwidth(uint16_t bytes_per_second)
{
    m_bandwidth = bytes_per_second;
}

cmav_stream_t *CompressedMavlinkEncoder::findStream(uint32_t msgid)
{
    for (uint8_t i = 0; i < m_num_streams; i++) {
        if (m_streams[i].mavlink_msgid == msgid) return &m_streams[i];
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Encode entry point
// ---------------------------------------------------------------------------

bool CompressedMavlinkEncoder::encode(uint32_t msgid, const uint8_t *payload,
                                       uint8_t payload_len, uint8_t *out_buf,
                                       uint8_t *out_len, uint32_t now_ms)
{
    *out_len = 0;

    cmav_stream_t *stream = findStream(msgid);
    if (!stream) return false;

    // Determine if we need a keyframe
    bool need_keyframe = stream->always_keyframe
                      || !stream->has_keyframe
                      || (stream->keyframe_interval_ms > 0 &&
                          (now_ms - stream->last_keyframe_ms) >= stream->keyframe_interval_ms);

    if (need_keyframe) {
        *out_len = encodeKeyframe(*stream, payload, out_buf, now_ms);
        return *out_len > 0;
    }

    // Delta encode
    *out_len = encodeDelta(*stream, payload, out_buf);
    return *out_len > 0;
}

// ---------------------------------------------------------------------------
// Keyframe encoding
// ---------------------------------------------------------------------------

uint8_t CompressedMavlinkEncoder::encodeKeyframe(cmav_stream_t &stream,
                                                   const uint8_t *payload,
                                                   uint8_t *out_buf,
                                                   uint32_t now_ms)
{
    uint8_t pos = 0;

    out_buf[pos++] = stream.stream_index;
    out_buf[pos++] = CMAV_FLAG_KEYFRAME;

    // Copy delta-encodable fields (from payload_offset onward)
    // For always-keyframe streams (HEARTBEAT etc), copy the entire payload
    uint8_t copy_offset = stream.always_keyframe ? 0 : stream.payload_offset;
    uint8_t copy_len = stream.raw_payload_size - copy_offset;

    memcpy(out_buf + pos, payload + copy_offset, copy_len);
    pos += copy_len;

    // Store last values for future delta computation
    if (!stream.always_keyframe && stream.num_fields > 0) {
        memcpy(stream.last_values, payload + stream.payload_offset,
               stream.raw_payload_size - stream.payload_offset);
    }

    stream.has_keyframe = true;
    stream.last_keyframe_ms = now_ms;

    return pos;
}

// ---------------------------------------------------------------------------
// Delta encoding
// ---------------------------------------------------------------------------

int32_t CompressedMavlinkEncoder::computeFloatDelta(const uint8_t *old_val,
                                                     const uint8_t *new_val,
                                                     float scale)
{
    float old_f, new_f;
    memcpy(&old_f, old_val, 4);
    memcpy(&new_f, new_val, 4);
    return (int32_t)roundf((new_f - old_f) / scale);
}

int32_t CompressedMavlinkEncoder::computeIntDelta(const uint8_t *old_val,
                                                    const uint8_t *new_val,
                                                    uint8_t field_size)
{
    int32_t old_v = 0, new_v = 0;
    // Little-endian, sign-extend for signed fields
    memcpy(&old_v, old_val, field_size);
    memcpy(&new_v, new_val, field_size);
    // Sign extend if 2-byte signed
    if (field_size == 2) {
        old_v = (int16_t)old_v;
        new_v = (int16_t)new_v;
    } else if (field_size == 1) {
        old_v = (int8_t)old_v;
        new_v = (int8_t)new_v;
    }
    return new_v - old_v;
}

uint8_t CompressedMavlinkEncoder::encodeDelta(cmav_stream_t &stream,
                                                const uint8_t *payload,
                                                uint8_t *out_buf)
{
    const uint8_t *new_fields = payload + stream.payload_offset;
    const uint8_t *old_fields = stream.last_values;

    // Compute deltas and determine bitmask
    int32_t deltas[CMAV_MAX_FIELDS_PER_STREAM];
    uint8_t bitmask = 0;
    uint8_t promote_bits = 0;
    bool any_changed = false;

    uint8_t field_offset = 0;
    for (uint8_t i = 0; i < stream.num_fields; i++) {
        int32_t delta;
        if (stream.float_scales[i] != 0.0f) {
            delta = computeFloatDelta(old_fields + field_offset,
                                       new_fields + field_offset,
                                       stream.float_scales[i]);
        } else {
            int32_t raw_delta = computeIntDelta(old_fields + field_offset,
                                                  new_fields + field_offset,
                                                  stream.field_sizes[i]);
            delta = (stream.int_scales[i] != 0) ? raw_delta / stream.int_scales[i] : raw_delta;
        }

        deltas[i] = delta;
        if (delta != 0) {
            bitmask |= (1 << i);
            any_changed = true;
            if (delta < -127 || delta > 127) {
                promote_bits |= (1 << i);
            }
        }
        field_offset += stream.field_sizes[i];
    }

    if (!any_changed) return 0;  // no output if nothing changed

    // Build the frame
    uint8_t pos = 0;
    out_buf[pos++] = stream.stream_index;

    uint8_t flags = 0;
    if (promote_bits) flags |= CMAV_FLAG_HAS_PROMOTE;
    out_buf[pos++] = flags;

    out_buf[pos++] = bitmask;

    if (promote_bits) {
        out_buf[pos++] = promote_bits;
    }

    // Pack deltas for changed fields
    for (uint8_t i = 0; i < stream.num_fields; i++) {
        if (!(bitmask & (1 << i))) continue;

        if (promote_bits & (1 << i)) {
            // int16
            int16_t d16 = (int16_t)deltas[i];
            memcpy(out_buf + pos, &d16, 2);
            pos += 2;
        } else {
            // int8
            out_buf[pos++] = (int8_t)deltas[i];
        }
    }

    // Update last_values with new field data
    memcpy(stream.last_values, new_fields,
           stream.raw_payload_size - stream.payload_offset);

    return pos;
}
