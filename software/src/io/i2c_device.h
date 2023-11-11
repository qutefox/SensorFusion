#pragma once

#include <stdint.h>

#include "i2c.h"

#include "src/io/i2c_device_interface.h"
#include "src/io/i2c_master_interface.h"

#define I2C_DEVICE_MAX_DATA_TX 64
#define I2C_DEVICE_ADDR_SIZE sizeof(uint8_t)
#define I2C_DEVICE_TX_BUF_SIZE (I2C_DEVICE_MAX_DATA_TX + I2C_DEVICE_ADDR_SIZE)

namespace io
{

class I2cDevice : public I2cDeviceInterface
{
    I2cMasterInterface* i2c_master = nullptr;
    mxc_i2c_req_t req;
    uint8_t buf[I2C_DEVICE_TX_BUF_SIZE];

protected:
    bool debug = false;

public:
    I2cDevice(uint8_t address, bool debug = false);
    virtual ~I2cDevice();

    virtual int begin() override;

    virtual int read_reg(uint8_t reg_addr, uint8_t& value) override;
    virtual int read_bytes(uint8_t reg_addr, uint8_t* value, unsigned int read_len) override;
    virtual int write_reg(uint8_t reg_addr, uint8_t value) override;
    virtual int write_bytes(uint8_t reg_addr, const uint8_t* value, unsigned int write_len) override;
};

} // namespace io
