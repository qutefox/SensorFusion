#pragma once

#include "src/io/digital_input_pin_interface.h"

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
    
    virtual int attach_interrupt1_handler(bool attached) = 0;
    virtual int attach_interrupt2_handler(bool attached) = 0;

public:
    SensorInterface()  
    { }
    virtual ~SensorInterface()
    { }

    virtual int begin() = 0;
    virtual int end() = 0;

    virtual int set_power_mode(PowerMode power_mode) = 0;

    virtual void set_interrupt_pin1(io::DigitalInputPinInterface* interrupt_pin) = 0;
    virtual void set_interrupt_pin2(io::DigitalInputPinInterface* interrupt_pin) = 0;

    virtual int handle_interrupt1() = 0;
    virtual int handle_interrupt2() = 0;
};

} // namespace sensor
