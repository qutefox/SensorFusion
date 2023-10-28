#pragma once

#include "gpio.h"

namespace io
{
namespace pin
{

class Output
{
    mxc_gpio_cfg_t gpio;

public:
    Output(mxc_gpio_regs_t *pin_port, uint32_t pin_mask);

    int set(bool value);
    int get(bool& value);
    int toggle();
};

} // namespace pin
} // namespace io
