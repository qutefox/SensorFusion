#include "i2c_slave.h"

#include "mxc_errors.h"
#include "mxc_lock.h"
#include "nvic_table.h"

#include "src/sensor_fusion_board.h"
#include "src/storage/register_map.h"
#include "src/debug_print.h"

namespace io
{
namespace i2c
{

I2cSlave* I2cSlave::instance = nullptr;
uint32_t I2cSlave::lock = 0;

I2cSlave::I2cSlave()
	: init_done{ false }
	, register_map{ storage::RegisterMap::get_instance() }
{

}

I2cSlave::~I2cSlave()
{
	NVIC_DisableIRQ(MXC_I2C_GET_IRQ(MXC_I2C_GET_IDX(I2C_SLAVE)));
	MXC_I2C_Shutdown(I2C_SLAVE);
	init_done = false;
}

I2cSlave* I2cSlave::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new I2cSlave();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int I2cSlave::begin()
{
	int err = E_NO_ERROR;

    if (init_done) return err;

	err = register_map->begin();
    if (err != E_NO_ERROR) return err;

	reset_state();

	// Initialise I2C slave
	err = MXC_I2C_Init(I2C_SLAVE, 0, I2C_SLAVE_ADDR);
	if (err != E_NO_ERROR) return err;

	err = MXC_I2C_SetFrequency(I2C_SLAVE, I2C_SLAVE_SPEED);
	if (err < 0) return err;

	// Enable I2C interrupt
	MXC_NVIC_SetVector(MXC_I2C_GET_IRQ(MXC_I2C_GET_IDX(I2C_SLAVE)), [](){ MXC_I2C_AsyncHandler(I2C_SLAVE); });
	NVIC_EnableIRQ(MXC_I2C_GET_IRQ(MXC_I2C_GET_IDX(I2C_SLAVE)));

	err = listen_for_next_event();
	if (err != E_NO_ERROR) return err;

	init_done = true;
    return err;
}

int I2cSlave::listen_for_next_event()
{
	int ret_val = MXC_I2C_SlaveTransactionAsync(I2C_SLAVE,
		[](mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal)->int
		{
			I2cSlave* i2c_slave = I2cSlave::get_instance();
			return i2c_slave->event_handler(i2c, event, retVal);
		});

	// debug_print("I2cSlave: listen_for_next_event called. return value: %d.\n", ret_val);

	return ret_val;
}

void I2cSlave::reset_state()
{
	// memset(rx_buf, 0x00, sizeof(rx_buf)); // Do we really need this?
	num_rx = 0;
	overflow = false;
	read_addr = 0;
	write_addr = 0;
	write_op = false;

	debug_print("reset_state\n");
}

int I2cSlave::event_handler(mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal)
{
	int err = *static_cast<int*>(retVal);

	debug_print("i2c slave event_handler: event=%d, retVal=%d.\n", event, err);

	switch(event)
	{
	case MXC_I2C_EVT_MASTER_WR:
		// A slave address match occurred with the master requesting a write to the slave.
		write_op = true;
		overflow = false;
		num_rx = 0;
		break;

	case MXC_I2C_EVT_MASTER_RD:
		// A slave address match occurred with the master requesting a read from the slave.
		
		debug_print("MXC_I2C_EVT_MASTER_RD");

		// Check if a read address was received
		if (write_op)
		{
			// Read all bytes from RXFIFO
			if (MXC_I2C_GetRXFIFOAvailable(I2C_SLAVE) != 0)
			{
				receive_data();
			}
			// Make sure we received a valid number of bytes to reset the read address.
			if (num_rx >= I2C_SLAVE_ADDR_SIZE)
			{
				read_addr = rx_buf[0];
			}
		}
		write_op = false;
		break;

	case MXC_I2C_EVT_UNDERFLOW:
		// The master has attempted a read when the transmit FIFO was empty.
	case MXC_I2C_EVT_TX_THRESH:
		// The transmit FIFO contains fewer bytes than its threshold level.

		// (Re)fill I2C TX FIFO
		send_data();
		break;

	case MXC_I2C_EVT_OVERFLOW:
		// The master has written data when the receive FIFO was already full.
	case MXC_I2C_EVT_RX_THRESH:
		// The receive FIFO contains more bytes than its threshold level.

		// Read data from I2C RX FIFO
		receive_data();
		break;

	case MXC_I2C_EVT_TRANS_COMP:
		//  The transaction has ended.

		// Transaction complete -> Reset state and process data as necessary
		cleanup(err);
		// listen_for_next_event(); -> returns E_BUSY over here
		break;

	default:
		return E_BAD_PARAM;
	}

	return E_NO_ERROR;
}

void I2cSlave::send_data()
{
	// Get the number of bytes available in the I2C TX FIFO
	// int tx_avail = MXC_I2C_GetTXFIFOAvailable(i2c);
	// TODO: use tx_avail

	uint8_t next_byte = register_map->read(read_addr);
	read_addr += MXC_I2C_WriteTXFIFO(I2C_SLAVE, &next_byte, 1);
}

void I2cSlave::receive_data()
{
	int rx_avail = MXC_I2C_GetRXFIFOAvailable(I2C_SLAVE);

	debug_print("receive_data: rx_avail=%d.\n", rx_avail);

	// Check whether receive buffer will overflow
	if ((rx_avail + num_rx) > static_cast<int>(I2C_SLAVE_RX_BUF_SIZE))
	{
		overflow = true;
		rx_avail -= MXC_I2C_ReadRXFIFO(I2C_SLAVE, &rx_buf[num_rx], I2C_SLAVE_RX_BUF_SIZE - num_rx);
		num_rx = I2C_SLAVE_ADDR_SIZE;
	}

	// Read remaining characters in FIFO
	num_rx += MXC_I2C_ReadRXFIFO(I2C_SLAVE, &rx_buf[num_rx], rx_avail);

	for (uint8_t i = 0 ; i < num_rx ; ++i)
	{
		debug_print("receive_data %d.) -> %02X.\n", i, rx_buf[i]);
	}
}

void I2cSlave::store_data()
{
	// Check to see if there are enough bytes to process received data
	if (num_rx < I2C_SLAVE_ADDR_SIZE) return;

	// Get number of data bytes received
	if (overflow) num_rx = I2C_SLAVE_MAX_DATA_RX;
	else num_rx -= I2C_SLAVE_ADDR_SIZE;

	// Get write address
	uint8_t write_addr = rx_buf[0];

	// Check write address
	if (!register_map->is_address_in_range(write_addr)) return;

	// Write bytes from buffer
	uint8_t rx_idx = I2C_SLAVE_ADDR_SIZE;
	for (uint8_t i = 0 ; i < num_rx; ++i)
	{
		register_map->write(write_addr+i, rx_buf[rx_idx+i]);
	}
}

void I2cSlave::cleanup(int err)
{
	debug_print("cleanup: err=%d.\n", err);
	if (err == E_NO_ERROR)
	{
		debug_print("cleanup: write op=%d.\n", write_op ? 1 : 0);
		if (write_op)
		{
			// Write operation
			// Read remaining characerts if any remain in FIFO
			int rx_len = MXC_I2C_GetRXFIFOAvailable(I2C_SLAVE);
			debug_print("cleanup: rx_len=%d.\n", rx_len);
			if(rx_len)
			{
				receive_data();
			}
			// Store received data
			store_data();
		}
		else
		{
			// Read operation
			// Decrement read counter if there are unsent data bytes
			read_addr -= MXC_I2C_FIFO_DEPTH - MXC_I2C_GetTXFIFOAvailable(I2C_SLAVE);

			while(I2C_SLAVE->status & MXC_F_I2C_STATUS_BUSY) { }
		}

		// Reset i2c slave state
		reset_state();
	}
}

} // namespace i2c
} // namespace io
