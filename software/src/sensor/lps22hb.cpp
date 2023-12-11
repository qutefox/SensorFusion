#include "lps22hb.h"

#include <cstring>

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/sensor/lps22hb-pid/lps22hb_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lps22hb* Lps22hb::instance = nullptr;
uint32_t Lps22hb::lock = 0;

Lps22hb::Lps22hb(uint8_t i2c_address, bool i2c_debug, io::DigitalInputPinInterface* _interrupt_pin)
    : SensorBase(i2c_address, i2c_debug)
    , interrupt_pin{ _interrupt_pin }
    , pressure_data_ready{ 0 }
    , temperature_data_ready{ 0 }
    , raw_pressure{ 0 }
    , raw_temperature{ 0 }
{
    
}

Lps22hb::~Lps22hb()
{
    
}

Lps22hb* Lps22hb::get_instance(uint8_t i2c_address, bool i2c_debug, io::DigitalInputPinInterface* interrupt_pin)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lps22hb(i2c_address, i2c_debug, interrupt_pin);
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Lps22hb::reset()
{
    err |= lps22hb_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (has_error()) return err;

    uint8_t rst = 1;
    do
    {
        err |= lps22hb_reset_get(dev_ctx, &rst);
        if (has_error()) return err;
    }
    while(rst);
    return err;
}

int Lps22hb::is_device_id_matching()
{
    uint8_t whoami = 0;
    err |= lps22hb_device_id_get(dev_ctx, &whoami);
    if (has_error()) return err;
    if (whoami != LPS22HB_ID) return E_NO_DEVICE;
    return E_NO_ERROR;
}

int Lps22hb::begin()
{
    err = E_NO_ERROR;
    err |= reset();

    err |= is_device_id_matching();
    if(has_error())
    {
        // No reason to go forward. We can give up here and now.
        register_map_helper->set_barometer_sensor_error(true);
        return err;
    }

    // Enable block data update.
    err |= lps22hb_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    // Enable i2c address auto increment.
    err |= lps22hb_auto_add_inc_set(dev_ctx, PROPERTY_ENABLE);
    // Route data ready interrupt to interrupt pin.
    err |= lps22hb_int_pin_mode_set(dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);
    // Enable data ready interrupt.
    err |= lps22hb_drdy_on_int_set(dev_ctx, PROPERTY_ENABLE);
    // Set interrupt pin polarity to active low.
    err |= lps22hb_int_polarity_set(dev_ctx, LPS22HB_ACTIVE_LOW);
    // Set low pass filter.
    err |= lps22hb_low_pass_filter_mode_set(dev_ctx, LPS22HB_LPF_ODR_DIV_2);
    // Set power mode.
    err |= set_power_mode(0, PowerMode::POWER_DOWN);
    // Configure interrupt handler.
    if (interrupt_pin != nullptr)
    {
        err |= interrupt_pin->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<Lps22hb*>(this_obj)->set_interrupt_active();
            }, this); // Passing the this pointer as the callback data.
    }

    register_map_helper->set_barometer_sensor_error(has_error());
    return err;
}

int Lps22hb::end()
{
    err = E_NO_ERROR;
    err |= reset();
    if (interrupt_pin != nullptr)
    {
        interrupt_pin->detach_interrupt_callback();
    }
    register_map_helper->set_barometer_sensor_error(has_error());
    return err;
}

int Lps22hb::set_power_mode(uint8_t device_index, PowerMode power_mode)
{
    if (device_index != 0) return E_NO_DEVICE;

    switch (power_mode)
    {
    default:
    case PowerMode::POWER_DOWN:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_POWER_DOWN);
        break;
    case PowerMode::LOW_POWER:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_10_Hz);
        break;
    case PowerMode::NORMAL:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_DISABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_25_Hz);
        break;
    case PowerMode::HIGH_PERFORMANCE:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_DISABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_50_Hz);
        break;
    }

    register_map_helper->set_barometer_sensor_error(has_error());

    if (power_mode != PowerMode::POWER_DOWN)
    {
        uint8_t press_drdy = 0;
        uint8_t temp_drdy = 0;
        lps22hb_data_ready_get(dev_ctx, &press_drdy, &temp_drdy);
        if (press_drdy || temp_drdy)
        {
            // Handle any available data so we can get the next interrupt.
            handle_interrupt();
        }
    }

    return err;
}

uint16_t Lps22hb::get_sample_rate_in_hz(uint8_t device_index)
{
    if (device_index != 0) return 0;
    
    lps22hb_odr_t odr;
    lps22hb_data_rate_get(dev_ctx, &odr);
    switch (odr)
    {
    case lps22hb_odr_t::LPS22HB_POWER_DOWN: return 0;
    case lps22hb_odr_t::LPS22HB_ODR_1_Hz:   return 1;
    case lps22hb_odr_t::LPS22HB_ODR_10_Hz:  return 10;
    case lps22hb_odr_t::LPS22HB_ODR_25_Hz:  return 25;
    case lps22hb_odr_t::LPS22HB_ODR_50_Hz:  return 50;
    case lps22hb_odr_t::LPS22HB_ODR_75_Hz:  return 75;
    }
    return 0;
}

int Lps22hb::handle_interrupt()
{
    err |= lps22hb_data_ready_get(dev_ctx, &pressure_data_ready, &temperature_data_ready);

    if (pressure_data_ready)
    {
        err |= lps22hb_pressure_raw_get(dev_ctx, &raw_pressure.u32bit);
        fusion_data->update_pressure(raw_pressure);
    }

    if (temperature_data_ready)
    {
        err |= lps22hb_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
        fusion_data->update_temperature(raw_temperature);
    }

    interrupt_active = false;
    register_map_helper->set_barometer_sensor_error(has_error());
    return err;
}