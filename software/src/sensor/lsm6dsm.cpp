#include "lsm6dsm.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/data_processor.h"
#include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lsm6dsm* Lsm6dsm::instance = nullptr;
uint32_t Lsm6dsm::lock = 0;

Lsm6dsm::Lsm6dsm(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* interrupt_pin1, io::DigitalInputPinInterface* interrupt_pin2)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2)
    , gyroscope_data_ready{ 0 }
    , accelerometer_data_ready{ 0 }
{

}

Lsm6dsm::~Lsm6dsm()
{
    
}

Lsm6dsm* Lsm6dsm::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::DigitalInputPinInterface* interrupt_pin1, io::DigitalInputPinInterface* interrupt_pin2)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lsm6dsm(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2);
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Lsm6dsm::reset()
{
    int err = lsm6dsm_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR) return err;

    uint8_t rst = 1;
    do
    {
        err = lsm6dsm_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR) return err;
    }
    while(rst);
    return err;
}

bool Lsm6dsm::is_device_id_valid()
{
    uint8_t whoami = 0;
    int err = lsm6dsm_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR || whoami != LSM6DSM_ID) return false;
    return true;
}

int Lsm6dsm::begin()
{
    int err = E_NO_ERROR;
    int err1 = E_NO_ERROR;
    int err2 = E_NO_ERROR;
    err |= reset();
    if(!is_device_id_valid())
    {
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
        return E_NO_DEVICE;
    }
     // Configure interrupt handler(s).
    err |= attach_interrupt1_handler(true);
    err |= attach_interrupt2_handler(true);
    // Set High Resolution Timestamp (25 us tick).
    err |= lsm6dsm_timestamp_res_set(dev_ctx, LSM6DSM_LSB_25us);
    // Enable timestamp in HW .
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
    int2_reg.int2_drdy_temp = PROPERTY_ENABLE;
    err |= lsm6dsm_pin_int2_route_set(dev_ctx, int2_reg);

    // Set power mode.
    err |= set_power_mode(PowerMode::POWER_DOWN);

    if (err != E_NO_ERROR)
    {
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
    }
    else
    {
        data_processor->set_gyroscope_sensor_error(err1 != E_NO_ERROR);
        data_processor->set_accelerometer_sensor_error(err2 != E_NO_ERROR);
    }
    return err || err1 || err2;
}

int Lsm6dsm::end()
{
    int err = E_NO_ERROR;

    err |= reset();
    err |= attach_interrupt1_handler(false);
    err |= attach_interrupt2_handler(false);
    if(!is_device_id_valid())
    {
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
        return E_NO_DEVICE;
    }
    err |= set_power_mode(PowerMode::POWER_DOWN);
    data_processor->set_gyroscope_sensor_error(err != E_NO_ERROR);
    data_processor->set_accelerometer_sensor_error(err != E_NO_ERROR);
    return err;
}

int Lsm6dsm::set_power_mode(PowerMode power_mode)
{
    int err = E_NO_ERROR;
    int err1 = E_NO_ERROR;
    int err2 = E_NO_ERROR;

    if (power_mode == PowerMode::HIGH_PERFORMANCE)
    {
        err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_HIGH_PERFORMANCE);
        err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_HIGH_PERFORMANCE);
    }
    else
    {
        err1 |= lsm6dsm_gy_power_mode_set(dev_ctx, LSM6DSM_GY_NORMAL);
        err2 |= lsm6dsm_xl_power_mode_set(dev_ctx, LSM6DSM_XL_NORMAL);
    }

    switch (power_mode)
    {
    default:
    case PowerMode::POWER_DOWN:
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_OFF);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_OFF);
        break;
    case PowerMode::LOW_POWER:
        // Low power for odrs: 12.5Hz, 26Hz and 52Hz.
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_52Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_52Hz);
        break;
    case PowerMode::NORMAL:
        // Normal for odrs: 104Hz and 208Hz.
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_208Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_104Hz);
        break;
    case PowerMode::HIGH_PERFORMANCE:
        // High performance for odrs: 416Hz, 833Hz, 1.66KHz, 3.33KHz and 6.66KHz.
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_833Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_416Hz);
        break;
    }

    if (err != E_NO_ERROR)
    {
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
    }
    else
    {
        data_processor->set_gyroscope_sensor_error(err1 != E_NO_ERROR);
        data_processor->set_accelerometer_sensor_error(err2 != E_NO_ERROR);
    }
    
    return err || err1 || err2;
}

int Lsm6dsm::handle_interrupt1()
{
    int err = lsm6dsm_data_ready_get(dev_ctx, &gyroscope_data_ready, &accelerometer_data_ready);
    if (err != E_NO_ERROR)
    {
        data_processor->set_gyroscope_sensor_error(true);
        data_processor->set_accelerometer_sensor_error(true);
        return err;
    }
    if (lsm6dsm_timestamp_raw_get(dev_ctx, &raw_timestamp.u32bit) == E_NO_ERROR)
    {
        data_processor->update_timestamp(raw_timestamp);
    }
    int accelerometer_err = E_NO_ERROR;
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
    int gyroscope_err = E_NO_ERROR;
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
    int err = lsm6dsm_temperature_raw_get(dev_ctx, &raw_temperature.i16bit);
    if (err == E_NO_ERROR)
    {
        data_processor->update_temperature(raw_temperature);
    }
    return err;
}
