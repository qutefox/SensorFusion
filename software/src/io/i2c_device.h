#pragma once

#include <stdint.h>

#include "i2c.h"

#define I2C_DEVICE_MAX_DATA_TX 64
#define I2C_DEVICE_ADDR_SIZE sizeof(uint8_t)
#define I2C_DEVICE_TX_BUF_SIZE (I2C_DEVICE_MAX_DATA_TX + I2C_DEVICE_ADDR_SIZE)

namespace io
{
namespace i2c
{

class I2cMaster; // Forward declare.

class I2cDevice
{
    I2cMaster* i2c_master = nullptr;
    mxc_i2c_req_t req;
    uint8_t buf[I2C_DEVICE_TX_BUF_SIZE];

protected:
    bool debug = false;

public:
    I2cDevice(uint8_t address, bool debug = false);

    int begin();

    int read_reg(uint8_t reg_addr, uint8_t& value);
    int read_bytes(uint8_t reg_addr, uint8_t* value, unsigned int read_len);
    int write_reg(uint8_t reg_addr, uint8_t value);
    int write_bytes(uint8_t reg_addr, const uint8_t* value, unsigned int write_len);
};

} // namespace i2c
} // namespace io
