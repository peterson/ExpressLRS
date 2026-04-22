# Compressed MAVLink — Detailed Implementation Plan

**Branch:** `compressed-mavlink`
**Prereq:** Read `compressed-mavlink-architecture.md` first.

---

## OTA constraints driving the design

| OTA Mode | Payload/packet | Typical rate | Effective bandwidth |
|---|---|---|---|
| OTA4 | 5 bytes | 50–100 Hz, 1:2 ratio | 125–250 B/s |
| OTA8 | 10 bytes | 150–500 Hz, 1:4 ratio | 375–1250 B/s |

Target: ≤10 bytes per delta frame (fits one OTA8 packet).
CRSF max payload: 62 bytes (keyframes must fit within this).

---

## MAVLink message priority hierarchy

### Priority 0 — Critical (always transmitted)

| Message | ID | Payload | Rate |
|---|---|---|---|
| HEARTBEAT | 0 | 9 B | 1 Hz |
| STATUSTEXT | 253 | ≤50 B | Event |
| COMMAND_ACK | 77 | 3 B | Event |

### Priority 1 — High (always flowing, delta-encoded)

| Message | ID | Payload | Nominal rate |
|---|---|---|---|
| ATTITUDE | 30 | 28 B | 10–20 Hz |
| GLOBAL_POSITION_INT | 33 | 28 B | 4–5 Hz |

### Priority 2 — Medium (rate-reduced under pressure)

| Message | ID | Payload | Nominal rate |
|---|---|---|---|
| SYS_STATUS | 1 | 31 B | 2 Hz |
| VFR_HUD | 74 | 20 B | 2 Hz |
| GPS_RAW_INT | 24 | 30 B | 1 Hz |
| BATTERY_STATUS | 147 | 36 B | 1 Hz |

### Priority 3 — Low (dropped first)

| Message | ID | Payload | Nominal rate |
|---|---|---|---|
| RC_CHANNELS | 65 | 42 B | 1 Hz |
| SERVO_OUTPUT_RAW | 36 | 21 B | 1 Hz |
| VIBRATION | 241 | 32 B | 1 Hz |
| EKF_STATUS_REPORT | 193 | 22 B | 1 Hz |

### Priority 4 — On-demand (dedicated bandwidth window)

| Message | ID | Direction |
|---|---|---|
| PARAM_VALUE | 22 | Downlink |
| PARAM_SET | 23 | Uplink |
| MISSION_ITEM_INT | 73 | Both |
| MISSION_COUNT / REQUEST / ACK | 44/51/47 | Both |
| COMMAND_LONG / COMMAND_INT | 76/75 | Uplink |

---

## Scale factors

The delta encoding uses per-field scale factors to map expected inter-frame
changes into int8 (±127). A single set of scale factors covers all vehicle
types from survey fixed-wing to FPV racing quad. The factors are sized for
worst-case dynamics (aggressive manoeuvres) — sedate vehicles simply produce
smaller delta values, which is fine. If a future use case needs tighter
precision for slow vehicles (e.g. camera georeferencing on a survey platform),
a second profile can be added; the codec structure supports it.

### ATTITUDE (msgid 30) — 6 delta-encodable fields

At 10 Hz (100 ms between frames):

| Field | Type | Scale | int8 range per delta | Rationale |
|---|---|---|---|---|
| roll | float | 0.01 rad/unit | ±73° | 500°/s snap roll = 50°/frame = 88 units, fits int8 |
| pitch | float | 0.01 rad/unit | ±73° | Same as roll |
| yaw | float | 0.02 rad/unit | ±146° | Rate-1 turn: 0.3°/frame = ~0.3 units. Fast yaw: 200°/s = 20°/frame = 17 units |
| rollspeed | float | 0.05 rad/s/unit | ±364°/s | Covers full acro range |
| pitchspeed | float | 0.05 rad/s/unit | ±364°/s | Same |
| yawspeed | float | 0.02 rad/s/unit | ±146°/s | Same |

`time_boot_ms` excluded from delta encoding — implicit from frame timing.

### GLOBAL_POSITION_INT (msgid 33) — 8 delta-encodable fields

At 5 Hz (200 ms between frames):

| Field | Type | Scale | int8 range per delta | Rationale |
|---|---|---|---|---|
| lat | int32 (degE7) | 30 /unit | ±42 m | 50 m/s = 10 m/frame = 3 units |
| lon | int32 (degE7) | 30 /unit | ±33 m (lat 35°) | Same |
| alt | int32 (mm) | 500 /unit | ±63.5 m | 15 m/s climb = 3 m/frame = 6 units |
| relative_alt | int32 (mm) | 500 /unit | ±63.5 m | Same |
| vx | int16 (cm/s) | 5 /unit | ±6.35 m/s | Covers acceleration transients |
| vy | int16 (cm/s) | 5 /unit | ±6.35 m/s | Same |
| vz | int16 (cm/s) | 10 /unit | ±12.7 m/s | Aggressive dive: 15 m/s Δ = 150 units → int16 promotion |
| hdg | uint16 (cdeg) | 50 /unit | ±63.5° | Fast turn: 20°/frame = 4 units |

`time_boot_ms` excluded.

### SYS_STATUS (msgid 1) — 4 delta-encodable fields

| Field | Type | Scale | int8 range | Rationale |
|---|---|---|---|---|
| voltage_battery | uint16 (mV) | 10 /unit | ±1.27 V | Changes slowly |
| current_battery | int16 (cA) | 1 /unit | ±1.27 A | Same |
| battery_remaining | int8 (%) | 1 /unit | ±127% | Direct fit |
| load | uint16 (‰) | 10 /unit | ±1.27 | Same |

### Quantisation error

Worst-case error per field = half the scale factor. All well within sensor accuracy:

| Field | Scale | Max error | Sensor accuracy |
|---|---|---|---|
| roll/pitch | 0.01 rad | 0.29° | Gyro ~0.01°, but GPS heading ~0.5° |
| lat/lon | 30 degE7 | 1.65 m | GPS ~2 m |
| alt | 500 mm | 25 cm | Baro ~0.5 m |

Keyframes every 2 seconds reset any accumulated quantisation drift.

---

## Compressed frame format

### CRSF encapsulation

```
Standard CRSF frame:
  [0xC8] [LEN] [0x80 = ARDUPILOT_RESP] [PAYLOAD...] [CRC]

Payload first byte = sub-type:
  0x00 = raw MAVLink passthrough (existing, no compression)
  0x01 = compressed MAVLink frame (this codec)

Compressed payload (after sub-type byte):
  [1] stream_index
  [1] frame_flags — bit 7: keyframe(1) / delta(0)
                     bit 6: has_promotion_bits
                     bits 0-5: reserved
  [N] frame body (keyframe or delta, see below)
```

### Keyframe body

```
  [N] raw field values in MAVLink byte order (no MAVLink header, no sysid/compid/seq)
```

### Delta body

```
  [1] field_bitmask   — which fields changed (1 bit per field, up to 8)
  [0-1] promotion_bits — present if frame_flags bit 6 set; which changed fields use int16
  [N] field_deltas    — int8 or int16, packed in field order for set bits only
```

### Window control frame

```
  [1] 0xFF (reserved stream index)
  [1] control_type: 0x01=window_open, 0x02=window_close, 0x03=keyframe_request
```

---

## Stream subscription table

```c
#define MAX_STREAMS           8
#define MAX_FIELDS_PER_STREAM 8
#define LAST_VALUES_BUF_SIZE  64

typedef struct {
    uint8_t       stream_index;
    uint32_t      mavlink_msgid;
    uint8_t       priority;             // 0=critical .. 4=on-demand
    uint8_t       nominal_rate_hz;
    uint8_t       current_rate_hz;      // adapted by scheduler
    uint8_t       num_fields;
    uint8_t       field_sizes[MAX_FIELDS_PER_STREAM];   // bytes per raw field
    int16_t       scale_factors[MAX_FIELDS_PER_STREAM]; // units per int8 step
    uint32_t      keyframe_interval_ms;
    uint32_t      last_keyframe_ms;
    uint32_t      last_sent_ms;
    uint8_t       last_values[LAST_VALUES_BUF_SIZE];
} stream_entry_t;
```

Default table:

| Index | Message | Priority | Rate | KF interval |
|---|---|---|---|---|
| 0 | HEARTBEAT | 0 | 1 Hz | always KF |
| 1 | ATTITUDE | 1 | 10 Hz | 2 s |
| 2 | GLOBAL_POSITION_INT | 1 | 5 Hz | 2 s |
| 3 | SYS_STATUS | 2 | 2 Hz | 2 s |
| 4 | VFR_HUD | 2 | 2 Hz | 2 s |
| 5 | GPS_RAW_INT | 2 | 1 Hz | 5 s |
| 6 | STATUSTEXT | 0 | event | always KF |
| 7 | COMMAND_ACK | 0 | event | always KF |

---

## Bandwidth savings — raw vs compressed

Per-message comparison at nominal stream rates:

| Message | Rate | Raw (with MAVLink header) | Compressed (delta + KF amortised) | Reduction |
|---|---|---|---|---|
| HEARTBEAT | 1 Hz | 17 B × 1 = 17 B/s | 11 B × 1 = 11 B/s | 35% (always KF) |
| ATTITUDE | 10 Hz | 40 B × 10 = 400 B/s | 6 B × 9.5 + 26 B × 0.5 = 70 B/s | **82%** |
| GLOBAL_POSITION_INT | 5 Hz | 40 B × 5 = 200 B/s | 7 B × 4.5 + 26 B × 0.5 = 45 B/s | **78%** |
| SYS_STATUS | 2 Hz | 43 B × 2 = 86 B/s | 4 B × 1.5 + 12 B × 0.5 = 12 B/s | 86% |
| VFR_HUD | 2 Hz | 32 B × 2 = 64 B/s | 5 B × 1.5 + 22 B × 0.5 = 19 B/s | 70% |
| GPS_RAW_INT | 1 Hz | 42 B × 1 = 42 B/s | 10 B × 1 = 10 B/s | 76% |
| **Total** | | **~809 B/s** | **~167 B/s** | **~79% (5× compression)** |

Keyframe amortisation: keyframes sent every 2 s (0.5 Hz) per stream. The
remaining frames are deltas. Delta sizes are typical for straight-and-level
flight; turns and manoeuvres produce slightly larger deltas but still
well within int8 range for most fields.

## Bandwidth budget by OTA configuration

### 250 Hz OTA8, 1:4 TLM ratio (~625 B/s available)

| | B/s |
|---|---|
| Compressed telemetry (all 6 streams) | ~167 |
| Headroom for STATUSTEXT, params, missions | ~458 |
| **Total available** | **625** |

### 150 Hz OTA8, 1:4 ratio (~375 B/s)

Full telemetry stream fits (167 B/s) with 208 B/s headroom.

### 50 Hz OTA4, 1:2 ratio (~125 B/s — worst case)

Drop medium-priority streams and reduce rates:
HEARTBEAT 1 Hz (11) + ATTITUDE 4 Hz (28) + POSITION 2 Hz (18) +
SYS_STATUS 0.5 Hz (4) = **~61 B/s**. Fits with 64 B/s headroom.

### 500 Hz OTA8, 1:2 ratio (~2500 B/s — best case)

Full telemetry with rate increases: ATTITUDE at 20 Hz, POSITION at 10 Hz.
Ample headroom for mission upload and parameter management.

---

## Implementation phases

### Phase 1 — Codec library + unit tests (~3 days)

New: `src/lib/CompressedMavlink/`

```
CompressedMavlinkDefs.h         — frame types, profile enums, constants
CompressedMavlinkStream.h       — stream_entry_t, scale factor tables per profile
CompressedMavlinkEncoder.h/cpp  — MAVLink bytes → compressed frame
CompressedMavlinkDecoder.h/cpp  — compressed frame → MAVLink bytes
CompressedMavlinkScheduler.h/cpp — priority scheduler, rate adaptation
```

**Encoder:**
```cpp
class CompressedMavlinkEncoder {
public:
    void init();
    bool encode(uint32_t msgid, const uint8_t *payload, uint8_t payload_len,
                uint8_t *out_buf, uint8_t *out_len, uint32_t now_ms);
    void resetAllStreams();
    void setBandwidth(uint16_t bytes_per_second);
};
```

**Decoder:**
```cpp
class CompressedMavlinkDecoder {
public:
    void init();
    bool decode(const uint8_t *compressed, uint8_t compressed_len,
                uint8_t *mavlink_out, uint8_t *mavlink_len);
    uint8_t requestKeyframe(uint8_t stream_index);
};
```

**Tests** (PlatformIO `native`, runs on host):

```
test_encoder.cpp      — encode known messages, verify output
test_decoder.cpp      — decode frames, verify MAVLink reconstruction
test_roundtrip.cpp    — encode → decode → compare (fuzz with random payloads)
test_delta.cpp        — per-message delta verification with known field changes
test_profiles.cpp     — verify SEDATE vs AGGRESSIVE scale factors produce
                        int8 for typical dynamics, int16 for extreme
test_scheduler.cpp    — priority ordering, rate adaptation under pressure
test_bandwidth.cpp    — simulated OTA rates, verify output stays within budget
```

### Phase 2 — RX integration (~2 days)

Insert encoder into `SerialMavlink.processBytes()`:

```cpp
void SerialMavlink::processBytes(uint8_t *bytes, uint16_t size)
{
    // Parse complete MAVLink frames from raw bytes
    for each complete MAVLink message:
        uint8_t compressed[64];
        uint8_t compressed_len;
        if (encoder.encode(msgid, payload, payload_len,
                           compressed, &compressed_len, millis()))
        {
            mavlinkInputBuffer.atomicPushBytes(compressed, compressed_len);
        }
}
```

`GetNextPayload()` wraps with sub-type 0x01:
```cpp
payloadData[0] = CRSF_ADDRESS_USB;
payloadData[1] = count + 1;         // +1 for sub-type byte
payloadData[2] = 0x01;              // compressed MAVLink sub-type
mavlinkInputBuffer.popBytes(payloadData + 3, count);
```

### Phase 3 — TX integration (~2 days)

Decoder in TX telemetry receive path:

```cpp
if (frame_type == CRSF_FRAMETYPE_ARDUPILOT_RESP)
{
    if (payload[0] == 0x01)  // compressed
    {
        uint8_t mavlink_buf[MAVLINK_MAX_PACKET_LEN];
        uint8_t mavlink_len;
        if (decoder.decode(payload + 1, payload_len - 1,
                           mavlink_buf, &mavlink_len))
        {
            forwardReconstructedMavlink(mavlink_buf, mavlink_len);
        }
    }
    else  // 0x00 = raw passthrough (backward compatible)
    {
        forwardRawMavlink(payload, payload_len);
    }
}
```

### Phase 4 — End-to-end software test (~2 days)

```
  [Skylight tlog]  →  Encoder  →  BW Limiter  →  Decoder  →  Compare
                                  (simulates OTA)
```

Test matrix:
- OTA4 @ 50 Hz, 1:2 (125 B/s) — worst case
- OTA8 @ 150 Hz, 1:4 (375 B/s) — typical
- OTA8 @ 250 Hz, 1:4 (625 B/s) — good
- OTA8 @ 500 Hz, 1:2 (2500 B/s) — best case

Each rate tested with both SEDATE and AGGRESSIVE profiles.

**Pass criteria:**
- Priority 0: 0% loss at all rates
- Priority 1: <5% loss at 150+ Hz, <20% at 50 Hz
- Priority 2: <10% loss at 250+ Hz
- Decoded field values within 1 scale-factor unit of original
- No buffer overflow

### Phase 5 — Hardware validation (~3 days)

Flash modified firmware, test with SITL → ELRS RX → OTA → ELRS TX → Skylight.

---

## Memory budget (ESP8285 — most constrained)

| Component | RAM |
|---|---|
| Stream table (8 × ~96 B) | 768 B |
| Encoder working buffer | 64 B |
| MAVLink parse state | 280 B |
| **Total codec overhead** | **~1.1 KB** |

Existing buffers reused. Compressed frames are smaller than raw MAVLink,
so buffer pressure decreases.

---

## Uplink (ground → vehicle)

Same codec handles uplink compression. TX encoder compresses COMMAND_LONG,
PARAM_SET, MISSION_ITEM_INT. RX decoder reconstructs and forwards to FC.

Mission upload via windowing: pause streaming, burst mission items with
per-item ACK, resume. ~20 seconds for 100 waypoints at 250 Hz OTA8.
