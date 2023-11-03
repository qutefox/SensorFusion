#pragma once

#include "src/io/input_pin.h"

namespace sensor
{

class SensorInterface
{
protected:
    virtual int reset() = 0;
    virtual inline void set_sensor_error1(bool value) = 0;
    virtual inline void set_sensor_error2(bool value) = 0;
    virtual bool is_device_id_valid() = 0;
    virtual int set_interrupt1_handler() = 0;
    virtual int set_interrupt2_handler() = 0;

public:
    SensorInterface()  
    { }
    virtual ~SensorInterface()
    { }

    virtual int begin() = 0;
    virtual int end() = 0;

    virtual void set_interrupt_pin1(io::pin::Input* interrupt_pin) = 0;
    virtual void set_interrupt_pin2(io::pin::Input* interrupt_pin) = 0;

    virtual int handle_interrupt1() = 0;
    virtual int handle_interrupt2() = 0;
};

} // namespace sensor
