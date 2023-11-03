#include "input_pin.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "lp.h"

using namespace io::pin;

Input::Input(mxc_gpio_regs_t *pin_port, uint32_t pin_mask, mxc_gpio_pad_t pullup_pulldown)
{
    gpio.port = pin_port;
    gpio.mask = pin_mask;
    gpio.pad = pullup_pulldown;
	gpio.func = MXC_GPIO_FUNC_IN;
}

int Input::begin()
{
    return MXC_GPIO_Config(&gpio);
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
        set_wake_up_enable(false);
        return err;
    }
    MXC_GPIO_EnableInt(gpio.port, gpio.mask);
    NVIC_EnableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
    set_wake_up_enable(true);
    return E_NO_ERROR;
}

void Input::detach_interrupt_callback()
{
    MXC_GPIO_DisableInt(gpio.port, gpio.mask);
    NVIC_DisableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
    set_wake_up_enable(false);
}

void Input::set_wake_up_enable(bool enabled)
{
    if (enabled)
    {
        MXC_LP_EnableGPIOWakeup(&gpio);
        MXC_GPIO_SetWakeEn(gpio.port, gpio.mask);
    }
    else
    {
        MXC_GPIO_ClearWakeEn(gpio.port, gpio.mask);
        MXC_LP_DisableGPIOWakeup(&gpio);
    }
}

bool Input::is_wake_up_enabled()
{
    return MXC_GPIO_GetWakeEn(gpio.port) & gpio.mask;
}
