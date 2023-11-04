#pragma once

#include <stdint.h>

#include "sensor_interface.h"
#include "src/io/i2c_device.h"
#include "src/data_processor_interface.h"

// Forward declare types.
struct _stmdev_ctx_t;
typedef _stmdev_ctx_t stmdev_ctx_t;

namespace sensor
{

class SensorBase : public SensorInterface
{
protected:
    bool init_done = false;
    DataProcessorInterface* data_processor = nullptr;
    io::i2c::I2cDevice* i2c_device = nullptr;
    stmdev_ctx_t* dev_ctx = nullptr;
    io::pin::Input* interrupt_pin1 = nullptr;
    io::pin::Input* interrupt_pin2 = nullptr;

    virtual int reset() = 0;
    virtual bool is_device_id_valid() = 0;

    virtual inline void set_sensor_error1(bool value) override
    {
        // Update appropriate register to indicate error.
    }

    virtual inline void set_sensor_error2(bool value) override
    {
        // Update appropriate register to indicate error.
    }

    virtual inline void set_sensor_errors(int err, int err1=false, int err2=false) override
    {
        if ((err != E_NO_ERROR) || (err1 != E_NO_ERROR)) set_sensor_error1(true);
        else set_sensor_error1(false);

        if ((err != E_NO_ERROR) || (err2 != E_NO_ERROR)) set_sensor_error2(true);
        else set_sensor_error2(false);
    }
    
    virtual inline int attach_interrupt1_handler(bool attached) override
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

    virtual inline int attach_interrupt2_handler(bool attached) override
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

public:
    SensorBase(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr, io::pin::Input* interrupt_pin2=nullptr);
    virtual ~SensorBase();

    virtual int begin() override;
    virtual int end() override;

    virtual int set_power_mode(PowerMode power_mode) = 0;

    virtual inline void set_interrupt_pin1(io::pin::Input* interrupt_pin) override
    {
        interrupt_pin1 = interrupt_pin;
    }

    virtual inline void set_interrupt_pin2(io::pin::Input* interrupt_pin) override
    {
        interrupt_pin2 = interrupt_pin;
    }
    
    virtual inline int handle_interrupt1() override
    {
        // Most probably read new sensor data and process/store it.
        return E_NO_ERROR;
    }

    virtual inline int handle_interrupt2() override
    {
        // Most probably read new sensor data and process/store it.
        return E_NO_ERROR;
    }
};

} // namespace sensor
