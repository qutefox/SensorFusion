#pragma once

#include <stdint.h>

#include "i2c.h"

#include "src/io/i2c_speed.h"

namespace io
{
namespace i2c
{

class I2cMaster
{
public:
    I2cMaster(i2c_speed_e speed);
    void scan();
    int transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size);
    int recover();
};

} // namespace i2c
} // namespace io
