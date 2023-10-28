#include "input_pin.h"

#include "mxc_device.h"
#include "mxc_errors.h"

#include "src/debug_print.h"

using namespace io::pin;

Input::Input(mxc_gpio_regs_t *pin_port, uint32_t pin_mask)
{
    gpio.port = pin_port;
    gpio.mask = pin_mask;
	gpio.pad = MXC_GPIO_PAD_NONE;
	gpio.func = MXC_GPIO_FUNC_IN;

	if (MXC_GPIO_Config(&gpio) != E_NO_ERROR)
    {
        debug_print("Failed to initialise input pin.\n");
    }
}

int Input::get(bool& value)
{
    value = false;
    if (MXC_GPIO_OutGet(gpio.port, gpio.mask)) value = true;
    return E_NO_ERROR;
}

int Input::attach_interrupt_callback(mxc_gpio_callback_fn func, void* cbdata, mxc_gpio_int_pol_t pol)
{
    MXC_GPIO_RegisterCallback(&gpio, func, cbdata);
    int err = MXC_GPIO_IntConfig(&gpio, pol);
    if (err != E_NO_ERROR)
    {
        debug_print("Failed to attach interrupt.\n");
        return err;
    }
    MXC_GPIO_EnableInt(gpio.port, gpio.mask);
    NVIC_EnableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
    return E_NO_ERROR;
}

void Input::detach_interrupt_callback()
{
    MXC_GPIO_DisableInt(gpio.port, gpio.mask);
    NVIC_DisableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
}