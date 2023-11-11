#pragma once

#include <stdint.h>

namespace io
{

class I2cDeviceInterface
{
public:
    I2cDeviceInterface() { }
    virtual ~I2cDeviceInterface() { }

    virtual int begin() = 0;

    virtual int read_reg(uint8_t reg_addr, uint8_t& value) = 0;
    virtual int read_bytes(uint8_t reg_addr, uint8_t* value, unsigned int read_len) = 0;
    virtual int write_reg(uint8_t reg_addr, uint8_t value) = 0;
    virtual int write_bytes(uint8_t reg_addr, const uint8_t* value, unsigned int write_len) = 0;
};

} // namespace io
