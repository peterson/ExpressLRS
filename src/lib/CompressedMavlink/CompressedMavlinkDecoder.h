#pragma once

#include "CompressedMavlinkDefs.h"

class CompressedMavlinkDecoder {
public:
    /**
     * Initialise decoder with default stream subscription table.
     * Must match the encoder's table for correct reconstruction.
     */
    void init();

    /**
     * Decode a compressed frame into a raw MAVLink payload.
     *
     * @param compressed      Compressed frame bytes (stream_index + flags + body)
     * @param compressed_len  Length of compressed data
     * @param mavlink_out     Output buffer for reconstructed payload (caller provides, ≥64 bytes)
     * @param mavlink_len     Set to reconstructed payload length on success
     * @return true if a valid MAVLink payload was reconstructed
     */
    bool decode(const uint8_t *compressed, uint8_t compressed_len,
                uint8_t *mavlink_out, uint8_t *mavlink_len);

    /**
     * Request a keyframe for a specific stream (e.g. if delta seems stale).
     * @return stream index to request, or 0xFF if invalid
     */
    uint8_t requestKeyframe(uint8_t stream_index);

private:
    cmav_stream_t m_streams[CMAV_MAX_STREAMS];
    uint8_t m_num_streams;

    cmav_stream_t *findStreamByIndex(uint8_t stream_index);
    void initDefaultStreams();

    bool decodeKeyframe(cmav_stream_t &stream, const uint8_t *body,
                        uint8_t body_len, uint8_t *mavlink_out, uint8_t *mavlink_len);

    bool decodeDelta(cmav_stream_t &stream, uint8_t flags,
                     const uint8_t *body, uint8_t body_len,
                     uint8_t *mavlink_out, uint8_t *mavlink_len);

    void applyIntDelta(uint8_t *field, uint8_t field_size, int32_t delta);
    void applyFloatDelta(uint8_t *field, float scale, int32_t delta_units);
};
