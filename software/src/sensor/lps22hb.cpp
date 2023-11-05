#include "lps22hb.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

Lps22hb* Lps22hb::instance = nullptr;
uint32_t Lps22hb::lock = 0;

Lps22hb::Lps22hb(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin)
    , fifo_buffer{ new lps22hb_fifo_output_data_t[32] }
{

}

Lps22hb::~Lps22hb()
{
    delete[] fifo_buffer;
}

Lps22hb* Lps22hb::get_instance(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin)
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

void Lps22hb::set_sensor_error1(bool value)
{
    data_processor->set_baro_sensor_error(value);
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
    err |= lps22hb_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    // Enable i2c address auto increment.
    err |= lps22hb_auto_add_inc_set(dev_ctx, PROPERTY_ENABLE);
    // Set fifo watermark to 25 samples.
    err |= lps22hb_fifo_watermark_set(dev_ctx, 25);
    // Set fifo mode to dynamic stream.
    err |= lps22hb_fifo_mode_set(dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);
    // Enable fifo.
    err |= lps22hb_fifo_set(dev_ctx, PROPERTY_ENABLE);
    // Enable interrupt when data stored in fifo reaches watermark (threshold) level.
    err |= lps22hb_fifo_threshold_on_int_set(dev_ctx, PROPERTY_ENABLE);
    // Put data ready or fifo flags on the interrupt pin.
    err |= lps22hb_int_pin_mode_set(dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);
    // Set interrupt pin mode to push pull.
    err |= lps22hb_pin_mode_set(dev_ctx, LPS22HB_PUSH_PULL);
    // Set interrupt pin polarity to active low.
    err |= lps22hb_int_polarity_set(dev_ctx, LPS22HB_ACTIVE_LOW);
    // Set low pass filter.
    err |= lps22hb_low_pass_filter_mode_set(dev_ctx, LPS22HB_LPF_ODR_DIV_2);
    // Set power mode.
    err |= set_power_mode(PowerMode::POWER_DOWN);

    set_sensor_error1(err);
    return err;
}

int Lps22hb::end()
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
        err |= lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_25_Hz);
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

    set_sensor_error1(err);
    return err;
}

int Lps22hb::handle_interrupt1()
{
    int err = E_NO_ERROR;

    uint8_t data_level = 0;
    err = lps22hb_fifo_data_level_get(dev_ctx, &data_level);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(err);
        return err;
    }

    err = lps22hb_fifo_output_data_burst_get(dev_ctx, fifo_buffer, data_level);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(err);
        return err;
    }

    int32_t avg_pressure = 0;
    int16_t avg_temperature = 0;

    int32_t modulo_avg_pressure = 0;
    int16_t modulo_avg_temperature = 0;

    int32_t curr_pressure = 0;
    int16_t curr_temperature = 0;

    // https://stackoverflow.com/questions/56663116/how-to-calculate-average-of-int64-t
    
    for (uint8_t i = 0 ; i < data_level ; ++i)
    {
        curr_pressure = lps22hb_fifo_output_data_to_raw_pressure(&fifo_buffer[i]);
        avg_pressure += curr_pressure / data_level;
        modulo_avg_pressure += curr_pressure % data_level;

        curr_temperature = lps22hb_fifo_output_data_to_raw_temperature(&fifo_buffer[i]);
        avg_temperature += curr_temperature / data_level;
        modulo_avg_temperature += curr_temperature % data_level;
    }

    avg_pressure += modulo_avg_pressure / data_level;
    raw_pressure.i32bit = avg_pressure;
    
    avg_temperature += modulo_avg_temperature / data_level;
    raw_temperature.i16bit = avg_temperature;

    data_processor->update_baro_data(raw_pressure, raw_temperature);

    set_sensor_error1(err);
    return err;
}