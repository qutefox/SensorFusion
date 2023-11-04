#include "output_pin.h"

#include "mxc_errors.h"

using namespace io::pin;

Output::Output(mxc_gpio_regs_t *pin_port, uint32_t pin_mask, mxc_gpio_drvstr_t drive_strength)
{
    gpio.port = pin_port;
    gpio.mask = pin_mask;
    gpio.drvstr = drive_strength;
	gpio.pad = MXC_GPIO_PAD_NONE;
	gpio.func = MXC_GPIO_FUNC_OUT;
}

int Output::begin()
{
    // TODO: Sometimes (especially after programmer reset) MXC_GPIO_Config fails with E_BAD_PARAM.
    // Maybe when MXC_GPIO_Config fails, we should try to reset then configure it again?
    return MXC_GPIO_Config(&gpio);
}

int Output::set(bool value)
{
    if (value) MXC_GPIO_OutSet(gpio.port, gpio.mask);
    else MXC_GPIO_OutClr(gpio.port, gpio.mask);
    return E_NO_ERROR;
}

int Output::get(bool& value)
{
    value = false;
    if (MXC_GPIO_OutGet(gpio.port, gpio.mask)) value = true;
    return E_NO_ERROR;
}

int Output::toggle()
{
    MXC_GPIO_OutToggle(gpio.port, gpio.mask);
    return E_NO_ERROR;
}
