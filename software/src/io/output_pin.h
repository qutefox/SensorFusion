#pragma once

#include "gpio.h"

namespace io
{
namespace pin
{

class Output
{
private:
    mxc_gpio_cfg_t gpio;

public:
    Output(mxc_gpio_regs_t *pin_port, uint32_t pin_mask,
        mxc_gpio_drvstr_t drive_strength = mxc_gpio_drvstr_t::MXC_GPIO_DRVSTR_0);
    virtual ~Output();

    int begin();

    int set(bool value);
    int get(bool& value);
    int toggle();
};

} // namespace pin
} // namespace io
