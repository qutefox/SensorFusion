#include "lis2mdl.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lis2mdl* Lis2mdl::instance = nullptr;
uint32_t Lis2mdl::lock = 0;

Lis2mdl::Lis2mdl(uint8_t i2c_address, bool i2c_debug, io::DigitalInputPinInterface* _interrupt_pin)
    : SensorBase(i2c_address, i2c_debug)
    , interrupt_pin{ _interrupt_pin }
{

}

Lis2mdl::~Lis2mdl()
{
    
}

Lis2mdl* Lis2mdl::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* interrupt_pin)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lis2mdl(i2c_address, i2c_debug, interrupt_pin);
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Lis2mdl::reset()
{
    err |= lis2mdl_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (has_error()) return err;

    uint8_t rst = 1;
    do
    {
        err |= lis2mdl_reset_get(dev_ctx, &rst);
        if (has_error()) return err;
    }
    while(rst);
    return err;
}

int Lis2mdl::is_device_id_matching()
{
    uint8_t whoami = 0;
    err |= lis2mdl_device_id_get(dev_ctx, &whoami);
    if (has_error()) return err;
    if (whoami != LIS2MDL_ID) return E_NO_DEVICE;
    return E_NO_ERROR;
}

int Lis2mdl::begin()
{
    err = E_NO_ERROR;
    err |= reset();

    err |= is_device_id_matching();
    if(has_error())
    {
        // No reason to go forward. We can give up here and now.
        register_map_helper->set_magnetometer_sensor_error(true);
        return err;
    }

    // Enable block data update.
    err |= lis2mdl_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    // Enable temperature compensation.
    err |= lis2mdl_offset_temp_comp_set(dev_ctx, PROPERTY_ENABLE);
    // Set Low-pass bandwidth to ODR/4.
    err |= lis2mdl_low_pass_bandwidth_set(dev_ctx, LIS2MDL_ODR_DIV_4);
    // Set/Reset sensor mode.
    err |= lis2mdl_set_rst_mode_set(dev_ctx, LIS2MDL_SET_SENS_ODR_DIV_63);
    // Enable interrupt generation on new data ready.
    err |= lis2mdl_drdy_on_pin_set(dev_ctx, PROPERTY_ENABLE);
    // Set power mode.
    err |= set_power_mode(0, PowerMode::POWER_DOWN);

    // Configure interrupt handler.
    if (interrupt_pin != nullptr)
    {
        err |= interrupt_pin->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<Lis2mdl*>(this_obj)->set_interrupt_active();
            }, this); // Passing the this pointer as the callback data.
    }

    register_map_helper->set_magnetometer_sensor_error(has_error());
    return err;
}

int Lis2mdl::end()
{
    err = E_NO_ERROR;
    err |= reset();
    if (interrupt_pin != nullptr)
    {
        interrupt_pin->detach_interrupt_callback();
    }
    register_map_helper->set_magnetometer_sensor_error(has_error());
    return err;
}

int Lis2mdl::set_power_mode(uint8_t device_index, PowerMode power_mode)
{
    if (device_index != 0) return E_NO_DEVICE;

    switch (power_mode)
    {
    default:
    case PowerMode::POWER_DOWN:
        err |= lis2mdl_power_mode_set(dev_ctx, LIS2MDL_LOW_POWER);
        err |= lis2mdl_operating_mode_set(dev_ctx, LIS2MDL_POWER_DOWN);
        break;
    case PowerMode::LOW_POWER:
        err |= lis2mdl_power_mode_set(dev_ctx, LIS2MDL_LOW_POWER);
        err |= lis2mdl_operating_mode_set(dev_ctx, LIS2MDL_CONTINUOUS_MODE);
        err |= lis2mdl_data_rate_set(dev_ctx, LIS2MDL_ODR_50Hz);
        break;
    case PowerMode::NORMAL:
        err |= lis2mdl_power_mode_set(dev_ctx, LIS2MDL_HIGH_RESOLUTION);
        err |= lis2mdl_operating_mode_set(dev_ctx, LIS2MDL_CONTINUOUS_MODE);
        err |= lis2mdl_data_rate_set(dev_ctx, LIS2MDL_ODR_100Hz);
        break;
    case PowerMode::HIGH_PERFORMANCE:
        err |= lis2mdl_power_mode_set(dev_ctx, LIS2MDL_HIGH_RESOLUTION);
        err |= lis2mdl_operating_mode_set(dev_ctx, LIS2MDL_CONTINUOUS_MODE);
        err |= lis2mdl_data_rate_set(dev_ctx, LIS2MDL_ODR_100Hz);
        break;
    }

    register_map_helper->set_magnetometer_sensor_error(has_error());

    if (power_mode != PowerMode::POWER_DOWN)
    {
        // Handle any available data so we can get the next interrupt.
        handle_interrupt();
    }

    return err;
}

uint16_t Lis2mdl::get_sample_rate_in_hz(uint8_t device_index)
{
    if (device_index != 0) return 0;
    
    lis2mdl_md_t md;
    lis2mdl_operating_mode_get(dev_ctx, &md);
    if (md == lis2mdl_md_t::LIS2MDL_POWER_DOWN) return 0;

    lis2mdl_odr_t odr;
    lis2mdl_data_rate_get(dev_ctx, &odr);
    switch (odr)
    {
    case lis2mdl_odr_t::LIS2MDL_ODR_10Hz:  return 10;
    case lis2mdl_odr_t::LIS2MDL_ODR_20Hz:  return 20;
    case lis2mdl_odr_t::LIS2MDL_ODR_50Hz:  return 50;
    case lis2mdl_odr_t::LIS2MDL_ODR_100Hz: return 100;
    }
    return 0;
}

int Lis2mdl::handle_interrupt()
{
    err |= lis2mdl_magnetic_raw_get(dev_ctx, raw_mag.i16bit);
    // The Fusion algorythm does not have any expectations regarding the unit.
    // So we can keep it in milli gauss.
    fusion_data->update_magnetometer(
        {
            lis2mdl_from_lsb_to_mgauss(raw_mag.i16bit[0]),
            lis2mdl_from_lsb_to_mgauss(raw_mag.i16bit[1]),
            lis2mdl_from_lsb_to_mgauss(raw_mag.i16bit[2])
        }
    );

    int temp_err = lis2mdl_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
    if (temp_err == E_NO_ERROR)
    {
        celsius = lis2mdl_from_lsb_to_celsius(raw_temperature.i16bit);
        raw_temperature.i16bit = static_cast<int16_t>(celsius*100);
        fusion_data->update_temperature(raw_temperature);
    }
    
    register_map_helper->set_magnetometer_sensor_error(has_error());
    return err;
}
