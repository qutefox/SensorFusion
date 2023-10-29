#pragma once

#include <stdint.h>

#include "i2c.h"

namespace io
{
namespace i2c
{

class I2cMaster
{
private:
    bool init_done = false;
    static I2cMaster* instance;
    static uint32_t lock;

protected:
    I2cMaster();
    ~I2cMaster();

public:
    I2cMaster(I2cMaster& other) = delete;
    void operator=(const I2cMaster& other) = delete;

    static I2cMaster* get_instance();

    int begin();
    
    void scan();
    int transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size);
    int recover();
};

} // namespace i2c
} // namespace io
