#include "lps22hb.h"

#include <cstring>

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

Lps22hb* Lps22hb::instance = nullptr;
uint32_t Lps22hb::lock = 0;

Lps22hb::Lps22hb(uint8_t i2c_address, bool i2c_debug, io::DigitalInputPinInterface* interrupt_pin)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin)
    , pressure_data_ready{ 0 }
    , temperature_data_ready{ 0 }
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
    int err = lps22hb_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR) return err;

    uint8_t rst = 1;
    do
    {
        err = lps22hb_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR) return err;
    }
    while(rst);
    return err;
}

bool Lps22hb::is_device_id_valid()
{
    uint8_t whoami = 0;
    int err = lps22hb_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR || whoami != LPS22HB_ID) return false;
    return true;
}

int Lps22hb::begin()
{
    int err = E_NO_ERROR;
    err |= reset();
    if(!is_device_id_valid())
    {
        data_processor->set_barometer_sensor_error(true);
        return E_NO_DEVICE;
    }

    // Configure interrupt handler.
    err |= attach_interrupt1_handler(true);
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
    err |= set_power_mode(PowerMode::POWER_DOWN);

    data_processor->set_barometer_sensor_error(err != E_NO_ERROR);
    return err;
}

int Lps22hb::end()
{
    int err = E_NO_ERROR;
    err |= reset();
    err |= attach_interrupt1_handler(false);
    if(!is_device_id_valid())
    {
        data_processor->set_barometer_sensor_error(true);
        return E_NO_DEVICE;
    }
    err |= set_power_mode(PowerMode::POWER_DOWN);
    data_processor->set_barometer_sensor_error(err != E_NO_ERROR);
    return err;
}

int Lps22hb::set_power_mode(PowerMode power_mode)
{
    int err = E_NO_ERROR;
    switch (power_mode)
    {
    default:
    case PowerMode::POWER_DOWN:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_POWER_DOWN);
        break;
    case PowerMode::LOW_POWER:
        err |= lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_1_Hz);
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

    data_processor->set_barometer_sensor_error(err != E_NO_ERROR);
    return err;
}

int Lps22hb::handle_interrupt1()
{
    int err = E_NO_ERROR;
     
    err |= lps22hb_data_ready_get(dev_ctx, &pressure_data_ready, &temperature_data_ready);

    if (pressure_data_ready)
    {
        err |= lps22hb_pressure_raw_get(dev_ctx, &raw_pressure.u32bit);
        data_processor->update_pressure(raw_pressure);
    }

    if (temperature_data_ready)
    {
        err |= lps22hb_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
        data_processor->update_temperature(raw_temperature);
    }

    data_processor->set_barometer_sensor_error(err != E_NO_ERROR);
    return err;
}