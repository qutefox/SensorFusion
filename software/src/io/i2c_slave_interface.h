#pragma once

namespace io
{

class I2cSlaveInterface
{
public:
    I2cSlaveInterface() { }
    virtual ~I2cSlaveInterface() { }

    virtual int begin(uint8_t slave_address) = 0;
    virtual bool is_current_transaction_done() const = 0;
};

} // namespace io
