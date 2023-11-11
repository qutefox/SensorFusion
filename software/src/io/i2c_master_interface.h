#pragma once

#include <stdint.h>

#include "i2c.h"

namespace io
{

class I2cMasterInterface
{
public:
    I2cMasterInterface() { }
    virtual ~I2cMasterInterface() { }

    virtual int begin() = 0;
    
    virtual void scan() = 0;
    virtual int transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size) = 0;
    virtual int recover() = 0;
};

} // namespace io
