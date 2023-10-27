#pragma once

#include "gpio.h"

namespace io
{

class PinIO
{
    mxc_gpio_cfg_t gpio;

public:
    PinIO(mxc_gpio_regs_t *pin_port, uint32_t pin_mask);

    int set(bool value);
    int get(bool& value);
    int toggle();
};

} // namespace io
