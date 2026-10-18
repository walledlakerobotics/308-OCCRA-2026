#include "utilities.hpp"
#include <cstring>
#include <xxhash.h>

void push_escaping(std::vector<uint8_t> &buffer, const std::vector<uint8_t> &data)
{
    buffer.reserve(buffer.size() + data.size() * 2); // Reserve space for worst-case scenario

    for (uint8_t byte : data)
    {
        if (byte == frame_signature || byte == escape_byte)
        {
            buffer.push_back(escape_byte);
            buffer.push_back(byte ^ 0x20); // XOR with 0x20 to escape the byte

            continue;
        }

        buffer.push_back(byte);
    }
}

std::vector<uint8_t> create_frame(const std::vector<uint8_t> &payload)
{
    // Reserve space for worst-case scenario
    std::vector<uint8_t> buffer;
    buffer.reserve(sizeof(frame_signature) + sizeof(size_t) * 2 + payload.size() * 2 + sizeof(XXH32_hash_t) * 2);

    buffer.push_back(frame_signature);

    // Length tag
    size_t payload_size = payload.size();

    push_escaping(buffer, to_bytes(payload_size));

    // Payload
    push_escaping(buffer, payload);

    // Checksum
    XXH32_hash_t checksum = XXH32(payload.data(), payload.size(), 0); // Calculate checksum using xxHash

    push_escaping(buffer, to_bytes(checksum));

    return buffer;
}