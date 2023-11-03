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

int SensorBase::end()
{
    // Detach interrupt handler.
    if (interrupt_pin1 != nullptr) interrupt_pin1->detach_interrupt_callback();
    if (interrupt_pin2 != nullptr) interrupt_pin2->detach_interrupt_callback();
    return E_NO_ERROR;
}

void SensorBase::set_interrupt_pin1(io::pin::Input* _interrupt_pin1)
{
    interrupt_pin1 = _interrupt_pin1;
}

void SensorBase::set_interrupt_pin2(io::pin::Input* _interrupt_pin2)
{
    interrupt_pin2 = _interrupt_pin2;
}

int SensorBase::handle_interrupt1()
{

}

int SensorBase::handle_interrupt2()
{

}

int SensorBase::set_interrupt1_handler()
{
    if (interrupt_pin1 != nullptr)
    {
        return interrupt_pin1->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<SensorBase*>(this_obj)->handle_interrupt1();
            }, this);
    }
    return E_NO_ERROR;
}

int SensorBase::set_interrupt2_handler()
{
    if (interrupt_pin2 != nullptr)
    {
        return interrupt_pin2->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<SensorBase*>(this_obj)->handle_interrupt2();
            }, this);
    }
    return E_NO_ERROR;
}
