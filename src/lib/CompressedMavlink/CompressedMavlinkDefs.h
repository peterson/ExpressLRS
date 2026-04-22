#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// CRSF sub-type byte (first byte of ARDUPILOT_RESP payload)
// ---------------------------------------------------------------------------

static const uint8_t CMAV_SUBTYPE_RAW        = 0x00;  // existing passthrough
static const uint8_t CMAV_SUBTYPE_COMPRESSED = 0x01;  // this codec

// ---------------------------------------------------------------------------
// Frame flag bits (second byte of compressed frame, after stream_index)
// ---------------------------------------------------------------------------

static const uint8_t CMAV_FLAG_KEYFRAME     = 0x80;  // bit 7: keyframe
static const uint8_t CMAV_FLAG_HAS_PROMOTE  = 0x40;  // bit 6: promotion bits present

// ---------------------------------------------------------------------------
// Stream / table limits
// ---------------------------------------------------------------------------

static const uint8_t CMAV_MAX_STREAMS           = 8;
static const uint8_t CMAV_MAX_FIELDS_PER_STREAM = 8;
static const uint8_t CMAV_LAST_VALUES_BUF_SIZE  = 64;

// Reserved stream index for window control frames
static const uint8_t CMAV_STREAM_WINDOW_CTRL = 0xFF;

// Window control types
static const uint8_t CMAV_WINDOW_OPEN     = 0x01;
static const uint8_t CMAV_WINDOW_CLOSE    = 0x02;
static const uint8_t CMAV_WINDOW_KF_REQ   = 0x03;

// ---------------------------------------------------------------------------
// Default keyframe interval
// ---------------------------------------------------------------------------

static const uint32_t CMAV_DEFAULT_KF_INTERVAL_MS = 2000;

// ---------------------------------------------------------------------------
// MAVLink message IDs (only the ones we compress)
// ---------------------------------------------------------------------------

static const uint32_t CMAV_MSGID_HEARTBEAT           = 0;
static const uint32_t CMAV_MSGID_SYS_STATUS          = 1;
static const uint32_t CMAV_MSGID_ATTITUDE             = 30;
static const uint32_t CMAV_MSGID_GLOBAL_POSITION_INT  = 33;
static const uint32_t CMAV_MSGID_COMMAND_ACK           = 77;
static const uint32_t CMAV_MSGID_VFR_HUD              = 74;
static const uint32_t CMAV_MSGID_GPS_RAW_INT          = 24;
static const uint32_t CMAV_MSGID_STATUSTEXT            = 253;

// ---------------------------------------------------------------------------
// Default stream indices
// ---------------------------------------------------------------------------

static const uint8_t CMAV_STREAM_HEARTBEAT  = 0;
static const uint8_t CMAV_STREAM_ATTITUDE   = 1;
static const uint8_t CMAV_STREAM_POSITION   = 2;
static const uint8_t CMAV_STREAM_SYS_STATUS = 3;
static const uint8_t CMAV_STREAM_VFR_HUD    = 4;
static const uint8_t CMAV_STREAM_GPS_RAW    = 5;
static const uint8_t CMAV_STREAM_STATUSTEXT = 6;
static const uint8_t CMAV_STREAM_CMD_ACK    = 7;

// ---------------------------------------------------------------------------
// Scale factors — one set covering all vehicle dynamics
// ---------------------------------------------------------------------------

// ATTITUDE (msgid 30) — 6 delta fields (time_boot_ms excluded)
static const float CMAV_SCALE_ATT_ROLL       = 0.01f;   // rad per unit
static const float CMAV_SCALE_ATT_PITCH      = 0.01f;
static const float CMAV_SCALE_ATT_YAW        = 0.02f;
static const float CMAV_SCALE_ATT_ROLLSPEED  = 0.05f;   // rad/s per unit
static const float CMAV_SCALE_ATT_PITCHSPEED = 0.05f;
static const float CMAV_SCALE_ATT_YAWSPEED   = 0.02f;

// GLOBAL_POSITION_INT (msgid 33) — 8 delta fields (time_boot_ms excluded)
static const int16_t CMAV_SCALE_POS_LAT      = 30;    // degE7 per unit
static const int16_t CMAV_SCALE_POS_LON      = 30;
static const int16_t CMAV_SCALE_POS_ALT      = 500;   // mm per unit
static const int16_t CMAV_SCALE_POS_RALT     = 500;
static const int16_t CMAV_SCALE_POS_VX       = 5;     // cm/s per unit
static const int16_t CMAV_SCALE_POS_VY       = 5;
static const int16_t CMAV_SCALE_POS_VZ       = 10;
static const int16_t CMAV_SCALE_POS_HDG      = 50;    // cdeg per unit

// SYS_STATUS (msgid 1) — 4 delta fields
static const int16_t CMAV_SCALE_SYS_VOLTAGE  = 10;    // mV per unit
static const int16_t CMAV_SCALE_SYS_CURRENT  = 1;     // cA per unit
static const int16_t CMAV_SCALE_SYS_REMAIN   = 1;     // % per unit
static const int16_t CMAV_SCALE_SYS_LOAD     = 10;    // permille per unit

// ---------------------------------------------------------------------------
// Stream entry — per-message codec state
// ---------------------------------------------------------------------------

struct cmav_stream_t {
    uint8_t   stream_index;
    uint32_t  mavlink_msgid;
    uint8_t   priority;                               // 0=critical .. 4=on-demand
    uint8_t   num_fields;                              // delta-encodable fields
    uint8_t   field_sizes[CMAV_MAX_FIELDS_PER_STREAM]; // bytes per raw field
    float     float_scales[CMAV_MAX_FIELDS_PER_STREAM]; // for float fields (0 = use int_scale)
    int16_t   int_scales[CMAV_MAX_FIELDS_PER_STREAM];   // for integer fields
    uint8_t   payload_offset;                          // offset to first delta field in raw payload
    uint8_t   raw_payload_size;                        // total raw payload size
    bool      always_keyframe;                         // true for HEARTBEAT, STATUSTEXT, CMD_ACK
    uint32_t  keyframe_interval_ms;
    uint32_t  last_keyframe_ms;
    bool      has_keyframe;                            // false until first keyframe sent
    uint8_t   last_values[CMAV_LAST_VALUES_BUF_SIZE]; // last transmitted field values
};
