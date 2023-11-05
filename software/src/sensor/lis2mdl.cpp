#include "lis2mdl.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/data_processor.h"
#include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lis2mdl* Lis2mdl::instance = nullptr;
uint32_t Lis2mdl::lock = 0;

Lis2mdl::Lis2mdl(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin)
{

}

Lis2mdl::~Lis2mdl()
{
    
}

Lis2mdl* Lis2mdl::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::pin::Input* interrupt_pin)
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
    int err = lis2mdl_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR) return err;

    uint8_t rst = 1;
    do
    {
        err = lis2mdl_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR) return err;
    }
    while(rst);
    return err;
}

void Lis2mdl::set_sensor_error1(bool value)
{
    data_processor->set_mag_sensor_error(value);
}

bool Lis2mdl::is_device_id_valid()
{
    uint8_t whoami = 0;
    int err = lis2mdl_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR || whoami != LIS2MDL_ID) return false;
    return true;
}

int Lis2mdl::begin()
{
    int err = E_NO_ERROR;
    if (init_done) return err;
    err |= reset();
    if(!is_device_id_valid())
    {
        set_sensor_error1(E_NO_DEVICE);
        return E_NO_DEVICE;
    }
    // Configure interrupt handler.
    err |= attach_interrupt1_handler(true);
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
    err |= set_power_mode(PowerMode::POWER_DOWN);

    set_sensor_error1(err);
    return err;
}

int Lis2mdl::end()
{
    int err = E_NO_ERROR;
    err |= reset();
    err |= attach_interrupt1_handler(false);
    if(!is_device_id_valid())
    {
        set_sensor_error1(E_NO_DEVICE);
        return E_NO_DEVICE;
    }
    err |= set_power_mode(PowerMode::POWER_DOWN);
    init_done = false;
    set_sensor_error1(err);
    return err;
}

int Lis2mdl::set_power_mode(PowerMode power_mode)
{
    int err = E_NO_ERROR;
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

    set_sensor_error1(err);
    return err;
}

int Lis2mdl::handle_interrupt1()
{
    int err = E_NO_ERROR;
    err |= lis2mdl_magnetic_raw_get(dev_ctx, raw_mag.i16bit);
    err |= lis2mdl_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
    
    data_processor->update_mag_data(raw_mag, raw_temperature);

    set_sensor_errors(err);
    return err;
}
