#include "pinio.h"

#include "mxc_errors.h"

#include "src/debug_print.h"

io::PinIO::PinIO(mxc_gpio_regs_t *pin_port, uint32_t pin_mask)
{
    gpio.port = pin_port;
    gpio.mask = pin_mask;
	gpio.pad = MXC_GPIO_PAD_NONE;
	gpio.func = MXC_GPIO_FUNC_OUT;

	if (MXC_GPIO_Config(&gpio) != E_NO_ERROR)
    {
        debug_print("Failed to initialise Led.\n");
    }
}

int io::PinIO::set(bool value)
{
    if (value) MXC_GPIO_OutSet(gpio.port, gpio.mask);
    else MXC_GPIO_OutClr(gpio.port, gpio.mask);
    return E_NO_ERROR;
}

int io::PinIO::get(bool& value)
{
    value = false;
    if (MXC_GPIO_OutGet(gpio.port, gpio.mask)) value = true;
    return E_NO_ERROR;
}

int io::PinIO::toggle()
{
    MXC_GPIO_OutToggle(gpio.port, gpio.mask);
    return E_NO_ERROR;
}
