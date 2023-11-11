#pragma once

#include "gpio.h"

namespace io
{

class DigitalInputPinInterface
{
public:
    DigitalInputPinInterface() { }
    virtual ~DigitalInputPinInterface() { }

    virtual int read(bool& value) = 0;
    virtual int attach_interrupt_callback(mxc_gpio_callback_fn func, void* cbdata, mxc_gpio_int_pol_t pol = MXC_GPIO_INT_FALLING) = 0;
    virtual void detach_interrupt_callback() = 0;

    virtual void set_wake_up_enable(bool enabled) = 0;
    virtual bool is_wake_up_enabled() = 0;
};

} // namespace io
