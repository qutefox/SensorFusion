#pragma once

#include <stdint.h>

namespace storage
{

class FlashInterface
{
public:
    virtual int read(uint8_t address, uint8_t* out_buffer, uint8_t length) = 0;
    virtual int write(uint8_t address, const uint8_t* in_buffer, uint8_t length) = 0;
};

} // namespace storage

