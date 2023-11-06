#include "lsm6dsm.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/data_processor.h"
#include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"
#include "src/debug_print.h"

using namespace sensor;

Lsm6dsm* Lsm6dsm::instance = nullptr;
uint32_t Lsm6dsm::lock = 0;

Lsm6dsm::Lsm6dsm(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2)
{

}

Lsm6dsm::~Lsm6dsm()
{
    
}

Lsm6dsm* Lsm6dsm::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
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

void Lsm6dsm::set_sensor_error1(bool value)
{
    data_processor->set_gyro_sensor_error(value);
}

void Lsm6dsm::set_sensor_error2(bool value)
{
    data_processor->set_accel_sensor_error(value);
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
    if (init_done) return err;
    err |= reset();
    if(!is_device_id_valid())
    {
        set_sensor_errors(E_NO_DEVICE);
        return E_NO_DEVICE;
    }
     // Configure interrupt handler(s).
    err |= attach_interrupt1_handler(true);
    err |= attach_interrupt2_handler(true);
    // Enable block data update.
    err |= lsm6dsm_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    // Enable i2c address auto increment.
    err |= lsm6dsm_auto_increment_set(dev_ctx, PROPERTY_ENABLE);
    // Set FIFO watermark to a multiple of a pattern.
    // Our pattern is: [GYRO + ACCEL + TEMP] = 6 + 6 + 2 bytes
    // We set watermark to 32 patterns.
    err |= lsm6dsm_fifo_watermark_set(dev_ctx, 32 * 14);
    err |=lsm6dsm_fifo_stop_on_wtm_set(dev_ctx, PROPERTY_ENABLE);
    // Set FIFO mode to Stream to FIFO.
    err |=lsm6dsm_fifo_mode_set(dev_ctx, LSM6DSM_STREAM_TO_FIFO_MODE);
    // Set FIFO sensor decimator.
    err1 |= lsm6dsm_fifo_gy_batch_set(dev_ctx, LSM6DSM_FIFO_GY_NO_DEC);
    err2 |= lsm6dsm_fifo_xl_batch_set(dev_ctx, LSM6DSM_FIFO_XL_NO_DEC);
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
    // Enable the temperature data storage in FIFO.
    err2 |= lsm6dsm_fifo_temp_batch_set(dev_ctx, PROPERTY_ENABLE);

    // Enable significant motion interrupt generation on INT1 pin.
    lsm6dsm_int1_route_t int1_reg;
    err |= lsm6dsm_pin_int1_route_get(dev_ctx, &int1_reg);
    int1_reg.int1_sign_mot = PROPERTY_ENABLE;
    err |= lsm6dsm_pin_int1_route_set(dev_ctx, int1_reg);

    // Enable FIFO watermark interrupt generation on INT2 pin.
    lsm6dsm_int2_route_t int2_reg;
    err |= lsm6dsm_pin_int2_route_get(dev_ctx, &int2_reg);
    int2_reg.int2_fth = PROPERTY_ENABLE;
    err |= lsm6dsm_pin_int2_route_set(dev_ctx, int2_reg);

    // Set power mode.
    err |= set_power_mode(PowerMode::POWER_DOWN);

    set_sensor_errors(err, err1, err2);
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
        set_sensor_errors(E_NO_DEVICE);
        return E_NO_DEVICE;
    }
    err |= set_power_mode(PowerMode::POWER_DOWN);
    init_done = false;
    set_sensor_errors(err);
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
        err |= lsm6dsm_fifo_data_rate_set(dev_ctx, LSM6DSM_FIFO_DISABLE);
        break;
    case PowerMode::LOW_POWER:
        // Low power for odrs: 12.5Hz, 26Hz and 52Hz.
        err |= lsm6dsm_fifo_data_rate_set(dev_ctx, LSM6DSM_FIFO_52Hz);
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_52Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_52Hz);
        break;
    case PowerMode::NORMAL:
        // Normal for odrs: 104Hz and 208Hz.
        err |= lsm6dsm_fifo_data_rate_set(dev_ctx, LSM6DSM_FIFO_208Hz);
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_208Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_208Hz);
        break;
    case PowerMode::HIGH_PERFORMANCE:
        // High performance for odrs: 416Hz, 833Hz, 1.66KHz, 3.33KHz and 6.66KHz.
        err |= lsm6dsm_fifo_data_rate_set(dev_ctx, LSM6DSM_FIFO_416Hz);
        err1 |= lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_416Hz);
        err2 |= lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_416Hz);
        break;
    }

    set_sensor_errors(err, err1, err2);
    return err || err1 || err2;
}

int Lsm6dsm::handle_interrupt1()
{
    int err = E_NO_ERROR;

    // Read number of word in FIFO.
    uint16_t num = 0;
    err |= lsm6dsm_fifo_data_level_get(dev_ctx, &num);

    uint16_t num_pattern = num / 14;
    while (num_pattern-- > 0)
    {
        err |= lsm6dsm_fifo_raw_data_get(dev_ctx, raw_gyro.u8bit, 3 * sizeof(int16_t));
        err |= lsm6dsm_fifo_raw_data_get(dev_ctx, raw_accel.u8bit, 3 * sizeof(int16_t));
        err |= lsm6dsm_fifo_raw_data_get(dev_ctx, raw_temperature.u8bit, sizeof(int16_t));
        data_processor->update_inertial_data(
            {
                lsm6dsm_from_fs500dps_to_mdps(raw_gyro.i16bit[0]),
                lsm6dsm_from_fs500dps_to_mdps(raw_gyro.i16bit[1]),
                lsm6dsm_from_fs500dps_to_mdps(raw_gyro.i16bit[2])
            },
            {
                lsm6dsm_from_fs2g_to_mg(raw_accel.i16bit[0]),
                lsm6dsm_from_fs2g_to_mg(raw_accel.i16bit[1]),
                lsm6dsm_from_fs2g_to_mg(raw_accel.i16bit[2])
            },
            raw_temperature);
    }

    set_sensor_errors(err);
    return err;
}

int Lsm6dsm::handle_interrupt2()
{
    int err = E_NO_ERROR;

    lsm6dsm_all_sources_t sources;
    lsm6dsm_all_sources_get(dev_ctx, &sources);
    if (sources.func_src1.sign_motion_ia)
    {
        // Significant motion event detected.
        // TODO: do something!
    }

    set_sensor_errors(err);
    return err;
}
