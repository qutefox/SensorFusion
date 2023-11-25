#pragma once

#include <stdint.h>
#include "src/io/digital_input_pin_interface.h"

namespace sensor
{

enum PowerMode
{
    POWER_DOWN = 0,
    LOW_POWER = 1,
    NORMAL = 2,
    HIGH_PERFORMANCE = 3
};

class SensorInterface
{
protected:
    virtual int reset() = 0;

public:
    SensorInterface() {}
    virtual ~SensorInterface() {}

    virtual int begin() = 0;
    virtual int end() = 0;

    virtual int set_power_mode(uint8_t device_index, PowerMode power_mode = PowerMode::POWER_DOWN) = 0;
    virtual bool has_error() = 0;
    virtual bool has_interrupt() = 0;
    virtual int handle_interrupt() = 0;
};

} // namespace sensor
