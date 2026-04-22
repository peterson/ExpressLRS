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

## Scale factor profiles

The delta encoding uses per-field scale factors to map expected inter-frame
changes into int8 (±127). The "right" scale factor depends on vehicle
dynamics — an FPV racing quad changes attitude 10× faster than a survey
fixed-wing. Using too-small scale factors wastes bandwidth on int16
promotions; too-large sacrifices precision.

### Profile selection

Two built-in profiles, selected at subscription time (hardcoded initially,
negotiable later via handshake):

| Profile | Use case | Attitude slew | Position slew | Typical vehicle |
|---|---|---|---|---|
| `SEDATE` | Survey, inspection, patrol | ≤5°/s roll/pitch, ≤15°/s yaw | ≤30 m/s, ≤3 m/s climb | Fixed-wing UAS, large multirotor |
| `AGGRESSIVE` | FPV racing, acro, rapid manoeuvre | ≤500°/s roll, ≤200°/s pitch/yaw | ≤50 m/s, ≤15 m/s climb | Racing quad, acro copter |

### ATTITUDE (msgid 30) — scale factors by profile

At 10 Hz (100 ms between frames):

| Field | SEDATE scale | SEDATE int8 range | AGGRESSIVE scale | AGGRESSIVE int8 range |
|---|---|---|---|---|
| roll | 0.001 rad/unit | ±7.3° per delta | 0.01 rad/unit | ±73° per delta |
| pitch | 0.001 rad/unit | ±7.3° | 0.01 rad/unit | ±73° |
| yaw | 0.005 rad/unit | ±36° | 0.02 rad/unit | ±146° |
| rollspeed | 0.002 rad/s/unit | ±14.5°/s | 0.05 rad/s/unit | ±364°/s |
| pitchspeed | 0.002 rad/s/unit | ±14.5°/s | 0.05 rad/s/unit | ±364°/s |
| yawspeed | 0.002 rad/s/unit | ±14.5°/s | 0.02 rad/s/unit | ±146°/s |

**SEDATE rationale:** A fixed-wing in a rate-1 turn (3°/s) changes heading
by 0.3° per 100 ms frame. Scale factor 0.005 rad/unit = 0.29°/unit → the
delta is ~1 unit, fitting comfortably in int8. Roll/pitch changes in cruise
are <0.5°/frame — again fits int8 trivially. Int16 promotion only triggers
during aggressive manoeuvres (wind gusts, turbulence recovery).

**AGGRESSIVE rationale:** An FPV quad doing a snap roll can hit 500°/s.
At 10 Hz that's 50°/frame. Scale factor 0.01 rad/unit = 0.57°/unit →
50°/0.57 = 88 units, still fits int8. A 720°/s snap roll would promote
to int16 — acceptable for an extreme manoeuvre that lasts <1 second.

### GLOBAL_POSITION_INT (msgid 33) — scale factors by profile

At 5 Hz (200 ms between frames):

| Field | SEDATE scale | SEDATE int8 range | AGGRESSIVE scale | AGGRESSIVE int8 range |
|---|---|---|---|---|
| lat (degE7) | 10 /unit | ±14 m | 30 /unit | ±42 m |
| lon (degE7) | 10 /unit | ±11 m (lat 35°) | 30 /unit | ±33 m |
| alt (mm) | 100 /unit | ±12.7 m | 500 /unit | ±63.5 m |
| relative_alt (mm) | 100 /unit | ±12.7 m | 500 /unit | ±63.5 m |
| vx (cm/s) | 1 /unit | ±1.27 m/s | 5 /unit | ±6.35 m/s |
| vy (cm/s) | 1 /unit | ±1.27 m/s | 5 /unit | ±6.35 m/s |
| vz (cm/s) | 1 /unit | ±1.27 m/s | 10 /unit | ±12.7 m/s |
| hdg (cdeg) | 10 /unit | ±12.7° | 50 /unit | ±63.5° |

**SEDATE rationale:** At 30 m/s ground speed, position changes by 6 m per
200 ms frame. Scale factor 10 degE7/unit → 6 m / 1.1 m/unit = ~5 units.
Fits int8. Velocity in cruise is nearly constant — delta is typically 0.
Altitude in level flight changes <0.5 m/frame (noise only).

**AGGRESSIVE rationale:** At 50 m/s with 5 g pull-up, altitude can change
by 15 m per frame. Scale 500 mm/unit → 15000/500 = 30 units, fits int8.
A racing quad in a dive at 40 m/s vertical: vz changes by ~15 m/s per frame
at 10 Hz — scale 10 cm/s/unit → 1500/10 = 150, promotes to int16. Acceptable
for a transient that lasts <2 seconds.

### SYS_STATUS (msgid 1) — same for both profiles

Battery and system health change slowly regardless of vehicle dynamics:

| Field | Scale | Range |
|---|---|---|
| voltage_battery (mV) | 10 /unit | ±1.27 V |
| current_battery (cA) | 1 /unit | ±1.27 A |
| battery_remaining (%) | 1 /unit | ±127% (always fits) |
| load (‰) | 10 /unit | ±1.27 (always fits) |

### Quantisation error analysis

The worst-case quantisation error per field is half the scale factor:

| Profile | Field | Scale | Max error | Acceptable? |
|---|---|---|---|---|
| SEDATE | roll | 0.001 rad | 0.0005 rad (0.03°) | GPS heading accuracy is ~0.5° |
| SEDATE | lat | 10 degE7 | 5 degE7 (0.55 m) | GPS accuracy is ~2 m |
| SEDATE | alt | 100 mm | 50 mm (5 cm) | Baro accuracy is ~0.5 m |
| AGGRESSIVE | roll | 0.01 rad | 0.005 rad (0.29°) | OSD resolution is ~1° |
| AGGRESSIVE | lat | 30 degE7 | 15 degE7 (1.65 m) | GPS accuracy is ~2 m |
| AGGRESSIVE | alt | 500 mm | 250 mm (25 cm) | Acceptable for FPV |

All errors are well within sensor accuracy. Keyframes every 2 seconds reset
any accumulated drift.

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
                     bits 4-5: profile (0=SEDATE, 1=AGGRESSIVE)
                     bits 0-3: reserved
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
#define MAX_STREAMS          8
#define MAX_FIELDS_PER_STREAM 8
#define LAST_VALUES_BUF_SIZE 64

typedef enum : uint8_t {
    PROFILE_SEDATE     = 0,
    PROFILE_AGGRESSIVE = 1,
} slew_profile_t;

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
    slew_profile_t profile;
} stream_entry_t;
```

Default table:

| Index | Message | Priority | Rate | KF interval | Profile |
|---|---|---|---|---|---|
| 0 | HEARTBEAT | 0 | 1 Hz | always KF | — |
| 1 | ATTITUDE | 1 | 10 Hz | 2 s | configurable |
| 2 | GLOBAL_POSITION_INT | 1 | 5 Hz | 2 s | configurable |
| 3 | SYS_STATUS | 2 | 2 Hz | 2 s | — |
| 4 | VFR_HUD | 2 | 2 Hz | 2 s | configurable |
| 5 | GPS_RAW_INT | 2 | 1 Hz | 5 s | — |
| 6 | STATUSTEXT | 0 | event | always KF | — |
| 7 | COMMAND_ACK | 0 | event | always KF | — |

---

## Bandwidth budget

### 250 Hz OTA8, 1:4 TLM ratio (~625 B/s available)

| Stream | Compressed size | Rate | B/s |
|---|---|---|---|
| HEARTBEAT | 10 B KF | 1 Hz | 10 |
| ATTITUDE | 6 B delta, 29 B KF | 10 Hz (0.5 Hz KF) | 75 |
| GLOBAL_POSITION_INT | 7 B delta, 29 B KF | 5 Hz (0.5 Hz KF) | 50 |
| SYS_STATUS | 4 B delta | 2 Hz (0.5 Hz KF) | 13 |
| VFR_HUD | 5 B delta | 2 Hz (0.5 Hz KF) | 15 |
| GPS_RAW_INT | 4 B delta | 1 Hz (0.2 Hz KF) | 10 |
| **Total** | | | **~173 B/s** |
| **Headroom** | | | **~450 B/s** |

### 50 Hz OTA4, 1:2 ratio (~125 B/s — worst case)

Drop to: HEARTBEAT (10) + ATTITUDE 4 Hz (28) + POSITION 2 Hz (18) +
SYS_STATUS 0.5 Hz (4) = **~60 B/s**. Still fits.

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
    void init(slew_profile_t profile = PROFILE_SEDATE);
    bool encode(uint32_t msgid, const uint8_t *payload, uint8_t payload_len,
                uint8_t *out_buf, uint8_t *out_len, uint32_t now_ms);
    void resetAllStreams();
    void setBandwidth(uint16_t bytes_per_second);
    void setProfile(slew_profile_t profile);  // update scale factors
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
