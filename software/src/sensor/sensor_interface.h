#pragma once

#include "src/io/input_pin.h"

namespace sensor
{

enum class PowerMode
{
    POWER_DOWN,
    LOW_POWER,
    NORMAL,
    HIGH_PERFORMANCE
};

class SensorInterface
{
protected:
    virtual int reset() = 0;
    virtual bool is_device_id_valid() = 0;
    
    virtual inline void set_sensor_error1(bool value) = 0;
    virtual inline void set_sensor_error2(bool value) = 0;
    virtual inline void set_sensor_errors(int err, int err1=false, int err2=false) = 0;
    
    virtual inline int attach_interrupt1_handler(bool attached) = 0;
    virtual inline int attach_interrupt2_handler(bool attached) = 0;

public:
    SensorInterface()  
    { }
    virtual ~SensorInterface()
    { }

    virtual int begin() = 0;
    virtual int end() = 0;

    virtual int set_power_mode(PowerMode power_mode) = 0;

    virtual inline void set_interrupt_pin1(io::pin::Input* interrupt_pin) = 0;
    virtual inline void set_interrupt_pin2(io::pin::Input* interrupt_pin) = 0;

    virtual inline int handle_interrupt1() = 0;
    virtual inline int handle_interrupt2() = 0;
};

} // namespace sensor
