#pragma once

#include "mxc_device.h"
#include "i2c.h"

#include "src/storage/register_map_reader_writer.h"

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

    uint8_t address;
	bool write_op; // Read/write operation state variable
	uint8_t num_rx; // Number of characters in rx_buf
	uint8_t rx_buf[I2C_SLAVE_RX_BUF_SIZE]; // Buffer to store received characters
    uint8_t tx_byte = 0;
	bool overflow; // Rx buffer overflowed during transaction
    int rx_avail = 0;
    volatile bool transaction_done = false;
    storage::RegisterMapReaderWriter* register_map_rw;

    int event_handler(mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal);

    void reset_state();

    void send_data();
    void receive_data();
    void store_data();
    void transaction_complete(int err);


protected:
    I2cSlave();
    ~I2cSlave();

public:
    I2cSlave(I2cSlave& other) = delete;
    void operator=(const I2cSlave& other) = delete;

    static I2cSlave* get_instance();

    int begin();

    int prepare_for_next_transaction();

    bool is_current_transaction_done() const { return transaction_done; }
};


} // namespace i2c
} // namespace io

