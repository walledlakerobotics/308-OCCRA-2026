#include <vector>
#include <cstdint>
#include <cstring>
#include <xxhash.h>

const uint8_t frame_signature = 0xAA;
const uint8_t escape_byte = 0xBB;

std::vector<uint8_t> create_frame(const std::vector<uint8_t> &payload);

void push_escaping(std::vector<uint8_t> &buffer, const std::vector<uint8_t> &data);

template <typename T>
void push_to_buffer(std::vector<uint8_t> &buffer, const T &data)
{
    size_t offset = buffer.size();

    if (offset + sizeof(T) > buffer.size())
    {
        buffer.resize(offset + sizeof(T));
    }

    memcpy(buffer.data() + offset, &data, sizeof(T));
}

template <typename T>
std::vector<uint8_t> to_bytes(const T &data)
{
    std::vector<uint8_t> bytes(sizeof(T));
    push_to_buffer(bytes, data);

    return bytes;
}