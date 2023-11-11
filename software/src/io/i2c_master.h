#pragma once

#include <stdint.h>

#include "i2c.h"

#include "src/io/i2c_master_interface.h"

namespace io
{

class I2cMaster : public I2cMasterInterface
{
private:
    static I2cMaster* instance;
    static uint32_t lock;

protected:
    I2cMaster();
    virtual ~I2cMaster();

public:
    I2cMaster(I2cMaster& other) = delete;
    void operator=(const I2cMaster& other) = delete;

    static I2cMaster* get_instance();

    virtual int begin() override;
    
    virtual void scan() override;
    virtual int transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size) override;
    virtual int recover() override;
};

} // namespace io
