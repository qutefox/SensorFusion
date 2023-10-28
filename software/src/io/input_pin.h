#pragma once

#include "gpio.h"

namespace io
{
namespace pin
{

class Input
{
    mxc_gpio_cfg_t gpio;

public:
    Input(mxc_gpio_regs_t *pin_port, uint32_t pin_mask);

    int get(bool& value);
    int attach_interrupt_callback(mxc_gpio_callback_fn func, void* cbdata, mxc_gpio_int_pol_t pol = MXC_GPIO_INT_FALLING);
    void detach_interrupt_callback();
};

} // namespace pin
} // namespace io
