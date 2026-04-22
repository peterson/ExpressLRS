#pragma once

#include "CompressedMavlinkDefs.h"

class CompressedMavlinkEncoder {
public:
    /**
     * Initialise encoder with default stream subscription table.
     */
    void init();

    /**
     * Encode a MAVLink message into a compressed frame.
     *
     * @param msgid     MAVLink message ID
     * @param payload   Raw MAVLink payload (no header, just fields)
     * @param payload_len  Length of payload in bytes
     * @param out_buf   Output buffer for compressed frame (caller provides, ≥64 bytes)
     * @param out_len   Set to compressed frame length on success
     * @param now_ms    Current time in milliseconds (for keyframe interval)
     * @return true if a compressed frame was produced, false if message was
     *         unknown, rate-limited, or had zero delta
     */
    bool encode(uint32_t msgid, const uint8_t *payload, uint8_t payload_len,
                uint8_t *out_buf, uint8_t *out_len, uint32_t now_ms);

    /**
     * Reset all stream state. Next encode for each stream will produce a keyframe.
     * Call on link reconnect.
     */
    void resetAllStreams();

    /**
     * Update available bandwidth. The scheduler uses this to adapt rates.
     * @param bytes_per_second  Estimated available telemetry bandwidth
     */
    void setBandwidth(uint16_t bytes_per_second);

private:
    cmav_stream_t m_streams[CMAV_MAX_STREAMS];
    uint8_t m_num_streams;
    uint16_t m_bandwidth;

    cmav_stream_t *findStream(uint32_t msgid);
    void initDefaultStreams();

    uint8_t encodeKeyframe(cmav_stream_t &stream, const uint8_t *payload,
                           uint8_t *out_buf, uint32_t now_ms);

    uint8_t encodeDelta(cmav_stream_t &stream, const uint8_t *payload,
                        uint8_t *out_buf);

    int32_t computeIntDelta(const uint8_t *old_val, const uint8_t *new_val,
                            uint8_t field_size);

    int32_t computeFloatDelta(const uint8_t *old_val, const uint8_t *new_val,
                              float scale);
};
