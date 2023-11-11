#include "sensor_base.h"

#include "src/data_processor.h"
#include "src/io/digital_input_pin_interface.h"
#include "src/io/i2c_device.h"
#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

SensorBase::SensorBase(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* interrupt_pin1,
    io::DigitalInputPinInterface* interrupt_pin2)
    : data_processor{ DataProcessor::get_instance() }
    , i2c_device{ new io::I2cDevice(i2c_address, i2c_debug) }
    , dev_ctx{ new stmdev_ctx_t() }
    , interrupt_pin1{ interrupt_pin1 }
    , interrupt_pin2{ interrupt_pin2 }
{
    i2c_device->begin();

    dev_ctx->handle = i2c_device;
    dev_ctx->write_reg =
        [](void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::I2cDeviceInterface* i2c_device = static_cast<io::I2cDeviceInterface*>(handle);
            return i2c_device->write_bytes(reg, bufp, len);
        };
    dev_ctx->read_reg =
        [](void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::I2cDeviceInterface* i2c_device = static_cast<io::I2cDeviceInterface*>(handle);
            return i2c_device->read_bytes(reg, bufp, len);
        };
}

SensorBase::~SensorBase()
{
    end();
    delete i2c_device;
    delete dev_ctx;
}

int SensorBase::end()
{
    return E_NO_ERROR;
}

int SensorBase::attach_interrupt1_handler(bool attached)
{
    if (interrupt_pin1 == nullptr) return E_NO_ERROR;
    if (attached)
    {
        return interrupt_pin1->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<SensorBase*>(this_obj)->handle_interrupt1();
            }, this);
    }
    interrupt_pin1->detach_interrupt_callback();
    return E_NO_ERROR;
}

int SensorBase::attach_interrupt2_handler(bool attached)
{
    if (interrupt_pin2 == nullptr) return E_NO_ERROR;
    if (attached)
    {
        return interrupt_pin2->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<SensorBase*>(this_obj)->handle_interrupt2();
            }, this);
    }
    interrupt_pin2->detach_interrupt_callback();
    return E_NO_ERROR;
}

void SensorBase::set_interrupt_pin1(io::DigitalInputPinInterface* interrupt_pin)
{
    interrupt_pin1 = interrupt_pin;
}

void SensorBase::set_interrupt_pin2(io::DigitalInputPinInterface* interrupt_pin)
{
    interrupt_pin2 = interrupt_pin;
}

int SensorBase::handle_interrupt1()
{
    // Most probably read new sensor data and process/store it.
    // You should override this method in your derived class.
    return E_NO_ERROR;
}

int SensorBase::handle_interrupt2()
{
    // Most probably read new sensor data and process/store it.
    // You should override this method in your derived class.
    return E_NO_ERROR;
}