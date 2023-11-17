#pragma once

#include "src/io/digital_input_pin_interface.h"

namespace io
{

class DigitalInputPin : public DigitalInputPinInterface
{
    mxc_gpio_cfg_t gpio;

public:
    DigitalInputPin(mxc_gpio_regs_t *pin_port, uint32_t pin_mask, mxc_gpio_pad_t pullup_pulldown = mxc_gpio_pad_t::MXC_GPIO_PAD_NONE);
    virtual ~DigitalInputPin();

    virtual int read(bool& value) override;
    virtual int attach_interrupt_callback(mxc_gpio_callback_fn func, void* cbdata, mxc_gpio_int_pol_t pol = MXC_GPIO_INT_FALLING) override;
    virtual void detach_interrupt_callback() override;

    virtual void set_wake_up_enable(bool enabled) override;
    virtual bool is_wake_up_enabled() override;

    virtual void set_pullup_pulldown(mxc_gpio_pad_t pullup_pulldown) override;
};

} // namespace io
