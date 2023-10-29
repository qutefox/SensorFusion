#pragma once

#include "mxc_device.h"
#include "i2c.h"

#include "src/storage/register_map.h"

#define I2C_SLAVE_MAX_DATA_RX 64
#define I2C_SLAVE_ADDR_SIZE sizeof(uint8_t)
#define I2C_SLAVE_RX_BUF_SIZE (I2C_SLAVE_MAX_DATA_RX + I2C_SLAVE_ADDR_SIZE)

namespace storage
{

class RegisterMap; // Forward declare.

} // namespace storage

namespace io
{
namespace i2c
{

class I2cSlave
{
private:
    bool init_done = false;
    static I2cSlave* instance;
    static uint32_t lock;

    uint8_t write_addr; // Address to start storing data at
	uint8_t read_addr; // Address to read data from
	bool write_op; // Read/write operation state variable
	uint8_t num_rx; // Number of characters in rx_buf
	uint8_t rx_buf[I2C_SLAVE_RX_BUF_SIZE]; // Buffer to store received characters
	bool overflow; // Rx buffer overflowed during transaction
    storage::RegisterMap* register_map;

    int listen_for_next_event();
    int event_handler(mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal);

    void reset_state();

    void send_data();
    void receive_data();
    void store_data();
    void cleanup(int err);


protected:
    I2cSlave();
    ~I2cSlave();

public:
    I2cSlave(I2cSlave& other) = delete;
    void operator=(const I2cSlave& other) = delete;

    static I2cSlave* get_instance();

    int begin();
};


} // namespace i2c
} // namespace io

