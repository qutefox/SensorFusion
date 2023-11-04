#include "sensor_base.h"

#include "src/data_processor.h"
#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

SensorBase::SensorBase(uint8_t i2c_address, bool i2c_debug,
    io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
    : init_done{ false }
    , data_processor{ DataProcessor::get_instance() }
    , i2c_device{ new io::i2c::I2cDevice(i2c_address, i2c_debug) }
    , dev_ctx{ new stmdev_ctx_t() }
    , interrupt_pin1{ interrupt_pin1 }
    , interrupt_pin2{ interrupt_pin2 }
{
    i2c_device->begin();

    dev_ctx->handle = i2c_device;
    dev_ctx->write_reg =
        [](void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = static_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->write_bytes(reg, bufp, len);
        };
    dev_ctx->read_reg =
        [](void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = static_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->read_bytes(reg, bufp, len);
        };
}

SensorBase::~SensorBase()
{
    if (init_done) end();
    delete i2c_device;
    delete dev_ctx;
}

int SensorBase::begin()
{
    int err = E_NO_ERROR;
    if (init_done) return err;
    set_sensor_errors(err != E_NO_ERROR);
    init_done = true;
    return err;
}

int SensorBase::end()
{
    set_sensor_errors(E_NO_ERROR);
    init_done = false;
    return E_NO_ERROR;
}