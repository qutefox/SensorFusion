#include "i2c_master.h"

#include "mxc_delay.h"

#include "src/debug_print.h"
#include "src/sensor_fusion_board.h"

namespace io
{
namespace i2c
{

I2cMaster::I2cMaster(i2c_speed_e speed)
{
    // Initialise I2C master
	if (MXC_I2C_Init(I2C_MASTER, 1, 0) != E_NO_ERROR)
	{
		debug_print("Failed to initialise I2C master.\n");
	}
    else
    {
        if (speed == i2c_speed_e::I2C_SPEED_100KHZ)
	        MXC_I2C_SetFrequency(I2C_MASTER, 100000u);
        else if (speed == i2c_speed_e::I2C_SPEED_400KHZ)
            MXC_I2C_SetFrequency(I2C_MASTER, 400000u);
    }
}

void I2cMaster::scan()
{
    mxc_i2c_req_t req;
    req.i2c = I2C_MASTER;
    req.addr = 0;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;

    debug_print("i2c master: starting scan..\n");
    uint8_t counter = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        req.addr = address;

        if (E_NO_ERROR == MXC_I2C_MasterTransaction(&req))
        {
            debug_print("i2c master: found slave device on address: 0x%02x\n", address);
            counter++;
        }
        MXC_Delay(MXC_DELAY_MSEC(50));
    }

    debug_print("i2c master: scan finished. %d many devices found.\n", counter);
}

int I2cMaster::transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size)
{
    req->i2c = I2C_MASTER;
    req->tx_buf = tx_data; // Write data buffer
    req->tx_len = tx_size; // Number of bytes to write
    req->rx_buf = rx_data; // Read data buffer
    req->rx_len = rx_size; // Number of bytes to read
    return MXC_I2C_MasterTransaction(req);
}

int I2cMaster::recover()
{
    return MXC_I2C_Recover(I2C_MASTER, 16);
}

} // namespace i2c
} // namespace io
