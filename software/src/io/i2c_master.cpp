#include "i2c_master.h"

#include "mxc_delay.h"
#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/debug_print.h"
#include "src/sensor_fusion_board.h"

using namespace io;

I2cMaster* I2cMaster::instance = nullptr;
uint32_t I2cMaster::lock = 0;

I2cMaster::I2cMaster()
    : init_done{ false }
{

}

I2cMaster::~I2cMaster()
{
    MXC_I2C_Shutdown(MXC_I2C_GET_I2C(I2C_MASTER));
}

I2cMaster* I2cMaster::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new I2cMaster();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int I2cMaster::begin()
{
    int err = E_NO_ERROR;
    if (init_done) return err;

    err = MXC_I2C_Init(MXC_I2C_GET_I2C(I2C_MASTER), 1, 0);
    if (err != E_NO_ERROR) return err;

    err = MXC_I2C_SetFrequency(MXC_I2C_GET_I2C(I2C_MASTER), I2C_MASTER_SPEED);
    if (err < 0) return err;

    err = MXC_I2C_SetClockStretching(MXC_I2C_GET_I2C(I2C_MASTER), I2C_MASTER_CLOCK_STRETCHING);
    if (err != E_NO_ERROR) return err;

    init_done = true;
    return err;
}

void I2cMaster::scan()
{
    mxc_i2c_req_t req;
    req.i2c = MXC_I2C_GET_I2C(I2C_MASTER);
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
            debug_print("i2c master: found slave device on address: 0x%02X\n", address);
            counter++;
        }
        MXC_Delay(MXC_DELAY_MSEC(50));
    }

    debug_print("i2c master: scan finished. %d many devices found.\n", counter);
}

int I2cMaster::transfer(mxc_i2c_req_t* req, uint8_t* tx_data, unsigned int tx_size, uint8_t* rx_data, unsigned int rx_size)
{
    req->i2c = MXC_I2C_GET_I2C(I2C_MASTER);
    req->tx_buf = tx_data; // Write data buffer
    req->tx_len = tx_size; // Number of bytes to write
    req->rx_buf = rx_data; // Read data buffer
    req->rx_len = rx_size; // Number of bytes to read
    return MXC_I2C_MasterTransaction(req);
}

int I2cMaster::recover()
{
    return MXC_I2C_Recover(MXC_I2C_GET_I2C(I2C_MASTER), 24);
}
