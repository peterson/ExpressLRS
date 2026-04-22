# Compressed MAVLink over ExpressLRS — Architecture

**Branch:** `compressed-mavlink`
**Status:** Design / not yet implemented
**Related:** [Skylight compressed-mavlink-elrs.md](https://github.com/peterson/skylight/blob/main/docs/plans/compressed-mavlink-elrs.md)

---

## Problem

ELRS telemetry bandwidth (0.3–3 KB/s depending on packet rate) is too low for
a full MAVLink stream (5–8 KB/s). The existing `SerialMavlink` passthrough in
ELRS chunks raw MAVLink into CRSF frames without compression, making it
impractical at typical ELRS rates.

## Goal

Full bidirectional MAVLink over ELRS with no additional RF hardware. A vehicle
with ELRS should be fully controllable from a GCS — telemetry, mission
upload/download, parameter management, commands — using only the existing ELRS
link. Target: ~8–9× compression on the telemetry downlink.

## Approach

Four-layer codec inserted between the MAVLink serial interface and the OTA
framing, on both the RX (vehicle) and TX (ground) sides.

```
EXISTING ELRS DATA PATH (RX side):

  FC UART → SerialMavlink.processBytes()
          → mavlinkInputBuffer (FIFO)
          → GetNextPayload() → raw MAVLink bytes → OTA frame

WITH COMPRESSED MAVLINK:

  FC UART → SerialMavlink.processBytes()
          → MAVLink parser (extract message fields)
          → CompressedMavlink encoder (delta/keyframe codec)
          → compressed payload → GetNextPayload() → OTA frame

TX SIDE (ground, mirror):

  OTA frame → compressed payload
            → CompressedMavlink decoder (reconstruct full MAVLink)
            → CRSF output to GCS (standard MAVLink, no compression visible)
```

## CRSF frame sub-typing

Compressed MAVLink frames are carried inside the existing
`CRSF_FRAMETYPE_ARDUPILOT_RESP` (0x80) frame type. The first byte of the
CRSF payload is a sub-type discriminator:

| Sub-type byte | Meaning |
|---|---|
| 0x00 | Raw MAVLink passthrough (existing behaviour, no compression) |
| 0x01 | Compressed MAVLink frame (this codec) |

A receiver that does not understand sub-type 0x01 will ignore the frame,
maintaining backward compatibility with unmodified ELRS firmware on either
side. Both sides must run the compressed-mavlink firmware for compression
to activate; if either side is stock ELRS, telemetry falls back to the
existing raw passthrough (sub-type 0x00).

## Codec layers

### Layer 1 — Header elimination

Both sides maintain a stream subscription table. Each active MAVLink message
stream gets a 1-byte index (0–255). The full MAVLink header (10 bytes: sysid,
compid, msgid, seq, etc.) is replaced by the stream index. Saves 9 bytes per
message.

### Layer 2 — Keyframe + delta encoding

Full keyframe every 2 seconds per stream. Between keyframes:
- 1-byte bitmask: which fields changed since last frame
- Changed fields as scaled int8 (±127) or int16 (±32767)
- Unchanged fields omitted entirely
- Promotion flag: int8 → int16 if delta exceeds int8 range

Example — ATTITUDE (28 bytes raw):
- Keyframe: 28 bytes (full IEEE 754 floats)
- Delta: ~6 bytes (1 index + 1 bitmask + ~4 int8 deltas)

### Layer 3 — Priority scheduling

| Priority | Messages | Rate | Behaviour |
|---|---|---|---|
| Critical | HEARTBEAT, STATUSTEXT | 1 Hz / event | Always transmitted |
| High | ATTITUDE, GLOBAL_POSITION_INT | 10–20 Hz | Delta-encoded, always flowing |
| Medium | SYS_STATUS, BATTERY, GPS_RAW, VFR_HUD | 1–2 Hz | Rate-reduced under pressure |
| Low | RC_CHANNELS, SERVO_OUTPUT, VIBRATION | 1 Hz | Dropped first under pressure |
| On-demand | PARAM_VALUE, MISSION_ITEM, COMMAND_ACK | Request/response | Dedicated bandwidth window |

### Layer 4 — Request/response windowing

Streaming telemetry pauses briefly (100–200 ms) for reliable exchanges:
mission upload, parameter read/write, command ACK. Per-frame ACK with retry.

## ELRS source references

The ExpressLRS project is fully open source (GPLv3). Key repositories:

| Repository | Purpose |
|---|---|
| [ExpressLRS/ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) | Main firmware (TX and RX). ESP32/ESP8285/STM32, PlatformIO build. |
| [ExpressLRS/Targets](https://github.com/ExpressLRS/Targets) | Hardware target definitions (pin mappings per board). |
| [ExpressLRS/ExpressLRS-Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator) | Cross-platform build/flash tool. |
| [ExpressLRS/Backpack](https://github.com/ExpressLRS/Backpack) | ESP-based backpack firmware (VRx, logging). |
| [ExpressLRS/ExpressLRS-Hardware](https://github.com/ExpressLRS/ExpressLRS-Hardware) | Open-source PCB designs. |

### ELRS MCU platforms

- **TX modules** (ground side, e.g. RadioMaster external module in JR bay): ESP32 / ESP32-S3
- **RX** (vehicle side): ESP8285 (most 2.4 GHz), some ESP32, some STM32
- **Radio ICs**: Semtech SX1280 (2.4 GHz), SX1276 (900 MHz), LR1121

The RadioMaster TX16S runs EdgeTX on the handset itself. ELRS runs on an
**external TX module** (ESP32 + SX1280) plugged into the JR module bay.

### Existing MAVLink-over-CRSF in ELRS

| File | Role |
|---|---|
| `src/src/rx-serial/SerialMavlink.cpp/.h` | RX-side: serial MAVLink interface, FIFO buffering (1 KB in, 512 B out), `GetNextPayload()` for OTA packing |
| `src/lib/CrsfProtocol/` | Core CRSF protocol: router, parser, connector, endpoint |
| `src/include/crsf_protocol.h` | CRSF frame types, addresses, protocol constants |
| `src/lib/Handset/CRSFHandset.cpp` | TX-side: receives CRSF telemetry, forwards to handset UART |

### Modification points

**RX side** — `SerialMavlink.cpp`: insert encoder between `processBytes()` and
the output FIFO. Parse MAVLink fields, compress, emit with sub-type 0x01.

**TX side** — telemetry receive path: detect sub-type 0x01 in
`CRSF_FRAMETYPE_ARDUPILOT_RESP`, decode via codec, output standard MAVLink.

**Shared codec** — new `src/lib/CompressedMavlink/` directory. Pure C++ with no
ELRS dependencies, portable to companion MCU (ESP32-S3 / RP2040).

## Implementation options

### Option A — External companion processor (recommended start)

Companion MCU (ESP32-S3 or RP2040) between FC and ELRS RX (vehicle side),
and between ELRS TX and GCS (ground side). No ELRS firmware changes.

### Option B — ELRS firmware fork (this branch)

Codec built directly into ELRS firmware. Lower latency, no extra hardware.
Requires maintaining the fork against upstream ELRS releases.

### Option C — Hybrid

Start with Option B for validation. Contribute upstream if the ELRS project
is receptive. Fall back to Option A if maintaining the fork is impractical.

## Relationship to Skylight

Skylight receives standard MAVLink from the ground-side TX module. The
compression is fully transparent to Phoenix — no Skylight changes required.
