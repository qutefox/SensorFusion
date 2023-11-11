#pragma once

namespace io
{

class I2cSlaveInterface
{
public:
    I2cSlaveInterface() { }
    virtual ~I2cSlaveInterface() { }

    virtual int begin() = 0;
    virtual bool is_current_transaction_done() const = 0;
};

} // namespace io
