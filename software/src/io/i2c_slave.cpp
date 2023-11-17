#include "i2c_slave.h"

#include "mxc_errors.h"
#include "mxc_lock.h"
#include "nvic_table.h"

#include "src/sensor_fusion_board.h"
#include "src/register_map.h"
#include "src/debug_print.h"

using namespace io;

I2cSlave* I2cSlave::instance = nullptr;
uint32_t I2cSlave::lock = 0;

I2cSlave::I2cSlave()
	: transaction_done{ false }
	, register_map{ Registermap::get_instance()->get_addressable_base() }
{

}

I2cSlave::~I2cSlave()
{
	NVIC_DisableIRQ(MXC_I2C_GET_IRQ(I2C_SLAVE));
	MXC_I2C_Shutdown(MXC_I2C_GET_I2C(I2C_SLAVE));
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

int I2cSlave::begin(uint8_t slave_address)
{
	int err = E_NO_ERROR;
	reset_state();

	// Initialise I2C slave
	err |= MXC_I2C_Init(MXC_I2C_GET_I2C(I2C_SLAVE), 0, slave_address);
	err = MXC_I2C_SetFrequency(MXC_I2C_GET_I2C(I2C_SLAVE), I2C_SLAVE_SPEED);
	if (err < 0) err |= E_FAIL;
	err |= MXC_I2C_SetClockStretching(MXC_I2C_GET_I2C(I2C_SLAVE), 1);

	// Enable I2C interrupt
	MXC_NVIC_SetVector(MXC_I2C_GET_IRQ(I2C_SLAVE),
		[]()
		{
			MXC_I2C_AsyncHandler(MXC_I2C_GET_I2C(I2C_SLAVE));

			I2cSlave* i2c_slave = I2cSlave::get_instance();
			if (i2c_slave->is_current_transaction_done())
			{
				i2c_slave->prepare_for_next_transaction();
			}
		});
	NVIC_EnableIRQ(MXC_I2C_GET_IRQ(I2C_SLAVE));

	err |= prepare_for_next_transaction();
    return err;
}

void I2cSlave::reset_state()
{
	num_rx = 0;
	overflow = false;
	write_op = false;
}

int I2cSlave::prepare_for_next_transaction()
{
	transaction_done = false;

	return MXC_I2C_SlaveTransactionAsync(MXC_I2C_GET_I2C(I2C_SLAVE),
		[](mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal)->int
		{
			I2cSlave* i2c_slave = I2cSlave::get_instance();
			return i2c_slave->event_handler(i2c, event, retVal);
		});
}

int I2cSlave::event_handler(mxc_i2c_regs_t* i2c, mxc_i2c_slave_event_t event, void* retVal)
{
	int err = *static_cast<int*>(retVal);

	// Check for valid params
    if (i2c != MXC_I2C_GET_I2C(I2C_SLAVE))
	{
        return E_INVALID;
    }

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

		write_op = false;
		overflow = false;

		// Read all bytes from RXFIFO.
		receive_data();

		// Make sure we received a valid number of bytes to reset the read address.
		if (num_rx >= I2C_SLAVE_ADDR_SIZE) address = rx_buf[0];
		
		break;

	case MXC_I2C_EVT_UNDERFLOW:
		// The master has attempted a read when the transmit FIFO was empty.
	case MXC_I2C_EVT_TX_THRESH:
		// The transmit FIFO contains fewer bytes than its threshold level.

		// (Re)fill I2C TX FIFO.
		send_data();
		break;

	case MXC_I2C_EVT_OVERFLOW:
		// The master has written data when the receive FIFO was already full.
	case MXC_I2C_EVT_RX_THRESH:
		// The receive FIFO contains more bytes than its threshold level.

		// Read data from I2C RX FIFO.
		receive_data();
		break;

	case MXC_I2C_EVT_TRANS_COMP:
		//  The transaction has ended.

		// Transaction complete -> Reset state and process data as necessary.
		transaction_complete(err);
		transaction_done = true;
		break;

	default:
		return E_BAD_PARAM;
	}

	return E_NO_ERROR;
}

void I2cSlave::send_data()
{
	if (register_map->is_valid_address(address))
	{
		register_map->read(address, tx_byte, true);
		address += MXC_I2C_WriteTXFIFO(MXC_I2C_GET_I2C(I2C_SLAVE), &tx_byte, 1);
	}
}

void I2cSlave::receive_data()
{
	int rx_avail = MXC_I2C_GetRXFIFOAvailable(MXC_I2C_GET_I2C(I2C_SLAVE));
	if (rx_avail == 0) return;

	// Check whether receive buffer will overflow.
	if ((rx_avail + num_rx) > static_cast<int>(I2C_SLAVE_RX_BUF_SIZE))
	{
		overflow = true;
		rx_avail -= MXC_I2C_ReadRXFIFO(MXC_I2C_GET_I2C(I2C_SLAVE), &rx_buf[num_rx], I2C_SLAVE_RX_BUF_SIZE - num_rx);
		num_rx = I2C_SLAVE_ADDR_SIZE;
	}

	// Read remaining characters in FIFO.
	num_rx += MXC_I2C_ReadRXFIFO(MXC_I2C_GET_I2C(I2C_SLAVE), &rx_buf[num_rx], rx_avail);
}

void I2cSlave::store_data()
{
	// Check to see if there are enough bytes to process received data.
	if (num_rx < I2C_SLAVE_ADDR_SIZE) return;

	// Get number of data bytes received
	if (overflow) num_rx = I2C_SLAVE_MAX_DATA_RX;
	else num_rx -= I2C_SLAVE_ADDR_SIZE;

	// Get write address
	address = rx_buf[0];

	// Is there any data or we just got an address?
	if (num_rx == 0) return;

	// Write bytes from buffer. Write address check is built into write.
	address += register_map->write(address, &rx_buf[I2C_SLAVE_ADDR_SIZE], num_rx, true, true);
}

void I2cSlave::transaction_complete(int err)
{
	if (err == E_NO_ERROR)
	{
		if (write_op) // Write operation:
		{
			// Read remaining characerts if any remain in FIFO.
			receive_data();

			// Store received data.
			store_data();
		}
		else // Read operation:
		{
			while(MXC_I2C_GET_I2C(I2C_SLAVE)->status & MXC_F_I2C_STATUS_BUSY) { }
		}
	}

	// Reset i2c slave state.
	reset_state();
}
