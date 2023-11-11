#pragma once

#include "gpio.h"

#include "src/io/digital_output_pin_interface.h"

namespace io
{

class DigitalOutputPin : public DigitalOutputPinInterface
{
private:
    mxc_gpio_cfg_t gpio;

public:
    DigitalOutputPin(mxc_gpio_regs_t *pin_port, uint32_t pin_mask,
        mxc_gpio_drvstr_t drive_strength = mxc_gpio_drvstr_t::MXC_GPIO_DRVSTR_0);
    virtual ~DigitalOutputPin();

    virtual int write(bool value) override;
    virtual int read(bool& value) override;
    virtual int toggle() override;
};

} // namespace io
