#include "i2c_device.h"

#include "src/io/i2c_master.h"
#include "src/debug_print.h"

using namespace io;

I2cDevice::I2cDevice(uint8_t address, bool _debug)
    : i2c_master{ I2cMaster::get_instance() }
    , debug{ _debug }
{
    req.addr = address;
    req.restart = 0;
    req.callback = nullptr;
}

I2cDevice::~I2cDevice()
{

}

int I2cDevice::begin()
{
    return i2c_master->begin();
}

int I2cDevice::read_reg(uint8_t reg_addr, uint8_t& value)
{
    if (debug) debug_print("I2C Device [0x%02x]: Reading register 0x%02x.\n", req.addr, reg_addr);

    int err = i2c_master->transfer(&req, &reg_addr, 1, &value, 1);

    if (err != E_NO_ERROR)
    {
        if (debug) debug_print("I2C Device [0x%02x]: Failed to read register 0x%02x. Error code: %d. Doing recovery then trying again.\n", req.addr, reg_addr, err);
        i2c_master->recover();
        err = i2c_master->transfer(&req, &reg_addr, 1, &value, 1);
        if (err != E_NO_ERROR)
        {
            if (debug) debug_print("I2C Device [0x%02x]: Failed to read register 0x%02x. Error code: %d. Recovery failed.\n", req.addr, reg_addr, err);
            return err;
        }
    }

    if (debug) debug_print("I2C Device [0x%02x]: Read register 0x%02x. Value: 0x%02x.\n", req.addr, reg_addr, value);
    return err;
}

int I2cDevice::read_bytes(uint8_t reg_addr, uint8_t* value, unsigned int read_len)
{
    if (debug) debug_print("I2C Device [0x%02x]: Reading registers 0x%02x. Read length: %d.\n", req.addr, reg_addr, read_len);

    int err = i2c_master->transfer(&req, &reg_addr, 1, value, read_len);

    if (err != E_NO_ERROR)
    {
        if (debug) debug_print("I2C Device [0x%02x]: Failed to read register 0x%02x. Read length: %d. Error code: %d. Doing recovery then trying again.\n", req.addr, reg_addr, read_len, err);
        i2c_master->recover();
        err = i2c_master->transfer(&req, &reg_addr, 1, value, read_len);
        if (err != E_NO_ERROR)
        {
            if (debug) debug_print("I2C Device [0x%02x]: Failed to read register 0x%02x. Read length: %d. Error code: %d. Recovery failed.\n", req.addr, reg_addr, read_len, err);
            return err;
        }
    }

    if (debug)
    {
        debug_print("I2C Device [0x%02x]: Read register 0x%02x.\n", req.addr, reg_addr);
        for(uint8_t i = 0 ; i < read_len ; ++i)
        {
            debug_print("I2C Device [0x%02x]: Read register 0x%02x value (%d) -> 0x%02x.\n", req.addr, reg_addr, i, value[i]);
        }
    }

    return err;
}

int I2cDevice::write_reg(uint8_t reg_addr, uint8_t value)
{
    buf[0] = reg_addr;
    buf[1] = value;

    if (debug) debug_print("I2C Device [0x%02x]: Writing register 0x%02x with value 0x%02x.\n", req.addr, reg_addr, buf[1]);

    int err = i2c_master->transfer(&req, &buf[0], 2, nullptr, 0);

    if (err != E_NO_ERROR)
    {
        if (debug) debug_print("I2C Device [0x%02x]: Failed to write register 0x%02x. Error code: %d. Doing recovery then trying again.\n", req.addr, err);
        i2c_master->recover();
        err = i2c_master->transfer(&req, &buf[0], 2, nullptr, 0);
        if (err != E_NO_ERROR)
        {
            if (debug) debug_print("I2C Device [0x%02x]: Failed to write register 0x%02x. Error code: %d. Recovery failed.\n", req.addr, err);
            return err;
        }
    }

    return err;
}

int I2cDevice::write_bytes(uint8_t reg_addr, const uint8_t* value, unsigned int write_len)
{
    if (debug) debug_print("I2C Device [0x%02x]: Writing register 0x%02x. Write length: %d.\n", req.addr, reg_addr, write_len);

    if ((write_len+1) > I2C_DEVICE_TX_BUF_SIZE)
    {
        if (debug) debug_print("I2C Device [0x%02x]: Failed to write register 0x%02x. Write length: %d. Write length is too big.\n", req.addr, reg_addr, write_len);
        return E_OVERFLOW;
    }

    buf[0] = reg_addr;
    for (uint8_t i = 0 ; i < write_len ; ++i)
    {
        buf[i+1] = value[i];
        if (debug) debug_print("I2C Device [0x%02x]: Writing register 0x%02x value (%d) -> 0x%02x.\n", req.addr, reg_addr, i, value[i]);
    }

    int err = i2c_master->transfer(&req, &buf[0], write_len + 1, nullptr, 0);

    if (err != E_NO_ERROR)
    {
        if (debug) debug_print("I2C Device [0x%02x]: Failed to write register 0x%02x. Error code: %d. Doing recovery then trying again.\n", req.addr, err);
        i2c_master->recover();
        err = i2c_master->transfer(&req, &buf[0], write_len + 1, nullptr, 0);
        if (err != E_NO_ERROR)
        {
            if (debug) debug_print("I2C Device [0x%02x]: Failed to write register 0x%02x. Error code: %d. Recovery failed.\n", req.addr, err);
            return err;
        }
    }

    return err;
}
