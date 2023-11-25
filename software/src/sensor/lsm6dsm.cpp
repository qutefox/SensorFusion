#include "lsm6dsm.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/processor_logic/data_processor.h"
#include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lsm6dsm* Lsm6dsm::instance = nullptr;
uint32_t Lsm6dsm::lock = 0;

Lsm6dsm::Lsm6dsm(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* _interrupt1_pin, io::DigitalInputPinInterface* _interrupt2_pin)
    : SensorBase(i2c_address, i2c_debug)
    , interrupt1_active{ false }
    , interrupt2_active{ false }
    , err1{ E_NO_ERROR }
    , err2{ E_NO_ERROR }
    , interrupt1_pin{ _interrupt1_pin }
    , interrupt2_pin{ _interrupt2_pin }
    , gyroscope_data_ready{ 0 }
    , accelerometer_data_ready{ 0 }
{

}

Lsm6dsm::~Lsm6dsm()
{
    
}

Lsm6dsm* Lsm6dsm::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* interrupt1_pin, io::DigitalInputPinInterface* interrupt2_pin)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lsm6dsm(i2c_address, i2c_debug, interrupt1_pin, interrupt2_pin);
    }
    MXC_FreeLock(&lock);
    return instance;
}

void Lsm6dsm::set_interrupt1_active()
{
    interrupt1_active = true;
}

void Lsm6dsm::set_interrupt2_active()
{
    interrupt2_active = true;
}

bool Lsm6dsm::has_interrupt()
{
    return interrupt_active || interrupt1_active || interrupt2_active;
}

bool Lsm6dsm::has_error()
{
    return (err != E_NO_ERROR) || (err1 != E_NO_ERROR) || (err2 != E_NO_ERROR);
}


int Lsm6dsm::reset()
{
    err |= lsm6dsm_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (has_error()) return err;

    uint8_t rst = 1;
    do
    {
        err |= lsm6dsm_reset_get(dev_ctx, &rst);
        if (has_error()) return err;
    }
    while(rst);
    return err;
}

int Lsm6dsm::is_device_id_matching()
{
    uint8_t whoami = 0;
    err |= lsm6dsm_device_id_get(dev_ctx, &whoami);
    if (has_error()) return err;
    if (whoami != LSM6DSM_ID) return E_NO_DEVICE;
    return E_NO_ERROR;
}

int Lsm6dsm::begin()
{
    err = E_NO_ERROR;
    err1 = E_NO_ERROR;
    err2 = E_NO_ERROR;
    err |= reset();
    
    err |= is_device_id_matching();
    if(has_error())
    {
        // No reason to go forward. We can give up here and now.
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
        return err;
    }

    // Set High Resolution Timestamp (25 us tick).
    err |= lsm6dsm_timestamp_res_set(dev_ctx, LSM6DSM_LSB_25us);
    // Enable timestamp in HW.
    err |= lsm6dsm_timestamp_set(dev_ctx, PROPERTY_ENABLE);
    // Enable block data update.
    err |= lsm6dsm_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    // Enable i2c address auto increment.
    err |= lsm6dsm_auto_increment_set(dev_ctx, PROPERTY_ENABLE);
    // Set interrupt pin mode to push pull.
    err |= lsm6dsm_pin_mode_set(dev_ctx, LSM6DSM_PUSH_PULL);
    // Set interrupt pin polarity to active low.
    err |= lsm6dsm_pin_polarity_set(dev_ctx, LSM6DSM_ACTIVE_LOW);
    // Set gyro full-scale.
    err1 |= lsm6dsm_gy_full_scale_set(dev_ctx, LSM6DSM_500dps);
    // Set gyro low pass bandwidth.
    err1 |= lsm6dsm_gy_band_pass_set(dev_ctx, LSM6DSM_LP2_ONLY);
    // Set accel full-scale.
    err2 |= lsm6dsm_xl_full_scale_set(dev_ctx, LSM6DSM_2g);
    // Set accel low pass filter 2.
    err2 |= lsm6dsm_xl_lp2_bandwidth_set(dev_ctx, LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_9);
    // Enable gyroscope, accelerometer interrupt generation on INT1 pin.
    lsm6dsm_int1_route_t int1_reg;
    err |= lsm6dsm_pin_int1_route_get(dev_ctx, &int1_reg);
    int1_reg.int1_drdy_g = PROPERTY_ENABLE;
    int1_reg.int1_drdy_xl = PROPERTY_ENABLE;
    err |= lsm6dsm_pin_int1_route_set(dev_ctx, int1_reg);
    // Enable temperature interrupt generation on INT2 pin.
    lsm6dsm_int2_route_t int2_reg;
    err |= lsm6dsm_pin_int2_route_get(dev_ctx, &int2_reg);
    // We could do some other fancy interrupts here. Maybe in the future..
    int2_reg.int2_drdy_temp = PROPERTY_ENABLE;
    err |= lsm6dsm_pin_int2_route_set(dev_ctx, int2_reg);
    // Set power mode.
    err1 |= set_power_mode(Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO, PowerMode::POWER_DOWN);
    err2 |= set_power_mode(Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL, PowerMode::POWER_DOWN);
    // Configure interrupt handler(s).
    if (interrupt1_pin != nullptr)
    {
        err |= interrupt1_pin->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<Lsm6dsm*>(this_obj)->set_interrupt1_active();
            }, this); // Passing the this pointer as the callback data.
    }

    if (interrupt2_pin != nullptr)
    {
        err |= interrupt2_pin->attach_interrupt_callback(
            [](void* this_obj) -> void
            {
                static_cast<Lsm6dsm*>(this_obj)->set_interrupt2_active();
            }, this); // Passing the this pointer as the callback data.
    }

    bool in_error_state = has_error();
    data_processor->set_gyroscope_sensor_error(in_error_state);
    data_processor->set_accelerometer_sensor_error(in_error_state);
    if (in_error_state) return E_FAIL;
    return E_NO_ERROR;
}

int Lsm6dsm::end()
{
    err = E_NO_ERROR;
    err1 = E_NO_ERROR;
    err2 = E_NO_ERROR;
    err |= reset();
    
    if (interrupt1_pin != nullptr)
    {
        interrupt1_pin->detach_interrupt_callback();
    }

    if (interrupt2_pin != nullptr)
    {
        interrupt2_pin->detach_interrupt_callback();
    }

    bool in_error_state = has_error();
    data_processor->set_gyroscope_sensor_error(in_error_state);
    data_processor->set_accelerometer_sensor_error(in_error_state);
    if (in_error_state) return E_FAIL;
    return E_NO_ERROR;
}

int Lsm6dsm::set_power_mode(uint8_t device_index, PowerMode power_mode)
{
    switch (power_mode)
    {
    default:
    case PowerMode::POWER_DOWN:
        if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO)
        {
            err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_OFF);
            err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_NORMAL);
        }
        else if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL)
        {
            err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_OFF);
            err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_NORMAL);
        }
        break;
    case PowerMode::LOW_POWER:
        // Low power for odrs: 12.5Hz, 26Hz and 52Hz.
        if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO)
        {
            err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_52Hz);
            err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_NORMAL);
        }
        else if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL)
        {
            err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_52Hz);
            err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_NORMAL);
        }
        break;
    case PowerMode::NORMAL:
        // Normal for odrs: 104Hz and 208Hz.
        if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO)
        {
            err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_208Hz);
            err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_NORMAL);
        }
        else if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL)
        {
            err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_208Hz);
            err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_NORMAL);
        }
        break;
    case PowerMode::HIGH_PERFORMANCE:
        // High performance for odrs: 416Hz, 833Hz, 1.66KHz, 3.33KHz and 6.66KHz.
        if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO)
        {
            err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_833Hz);
            err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_HIGH_PERFORMANCE);
        }
        else if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL)
        {
            err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_833Hz);
            err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_HIGH_PERFORMANCE);
        }
        break;
    }

    if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO)
    {
        bool gyroscope_err = (err != E_NO_ERROR) || (err1 != E_NO_ERROR);
        data_processor->set_gyroscope_sensor_error(gyroscope_err);
        return gyroscope_err;
    }
    else if (device_index == Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL)
    {
        bool accelerometer_err = (err != E_NO_ERROR) || (err2 != E_NO_ERROR);
        data_processor->set_accelerometer_sensor_error(accelerometer_err);
        return accelerometer_err;
    }

    return E_NO_DEVICE;
}

int Lsm6dsm::handle_interrupt()
{
    int ret_val = E_NO_ERROR;
    if (interrupt1_active)
    {
        ret_val |= handle_interrupt1();
        interrupt1_active = false;
    }

    if (interrupt2_active)
    {
        ret_val |= handle_interrupt2();
        interrupt2_active = false;
    }
    return ret_val;
}

int Lsm6dsm::handle_interrupt1()
{
    if (lsm6dsm_timestamp_raw_get(dev_ctx, &raw_timestamp.u32bit) == E_NO_ERROR)
    {
        data_processor->update_timestamp(raw_timestamp);
    }

    gyroscope_data_ready = false;
    accelerometer_data_ready = false;
    err |= lsm6dsm_data_ready_get(dev_ctx, &gyroscope_data_ready, &accelerometer_data_ready);

    int accelerometer_err = E_NO_ERROR;
    int gyroscope_err = E_NO_ERROR;

    if (accelerometer_data_ready)
    {
        accelerometer_err = lsm6dsm_acceleration_raw_get(dev_ctx, raw_accelerometer.i16bit);
        if (accelerometer_err == E_NO_ERROR)
        {
            data_processor->update_accelerometer_fusion_vector(
                {
                    lsm6dsm_from_fs2g_to_mg(raw_accelerometer.i16bit[0]),
                    lsm6dsm_from_fs2g_to_mg(raw_accelerometer.i16bit[1]),
                    lsm6dsm_from_fs2g_to_mg(raw_accelerometer.i16bit[2])
                }
            );
        }
        data_processor->set_accelerometer_sensor_error(accelerometer_err != E_NO_ERROR);
    }

    if (gyroscope_data_ready)
    {
        gyroscope_err = lsm6dsm_angular_rate_raw_get(dev_ctx, raw_gyroscope.i16bit);
        if (gyroscope_err == E_NO_ERROR)
        {
            data_processor->update_gyroscope_fusion_vector(
                {
                    lsm6dsm_from_fs500dps_to_mdps(raw_gyroscope.i16bit[0]),
                    lsm6dsm_from_fs500dps_to_mdps(raw_gyroscope.i16bit[1]),
                    lsm6dsm_from_fs500dps_to_mdps(raw_gyroscope.i16bit[2])
                }
            );
        }
        data_processor->set_gyroscope_sensor_error(gyroscope_err != E_NO_ERROR);
    }

    return gyroscope_err || accelerometer_err;
}

int Lsm6dsm::handle_interrupt2()
{
    int temp_err = lsm6dsm_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
    if (temp_err == E_NO_ERROR)
    {
        data_processor->update_temperature(raw_temperature);
    }
    return temp_err;
}
