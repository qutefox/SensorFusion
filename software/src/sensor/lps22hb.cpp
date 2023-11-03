#include "lps22hb.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

Lps22hb* Lps22hb::instance = nullptr;
uint32_t Lps22hb::lock = 0;

Lps22hb::Lps22hb(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2)
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
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    uint8_t rst;
    do
    {
        err = lps22hb_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            set_sensor_error1(true);
            return err;
        }
    }
    while(rst);
    return err;
}

void Lps22hb::set_sensor_error1(bool value)
{
    data_processor->set_baro_sensor_error(value);
}

void Lps22hb::set_sensor_error2(bool value)
{
}

bool Lps22hb::is_device_id_valid()
{
    uint8_t whoami = 0;
    int err = lps22hb_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR || whoami != LPS22HB_ID)
    {
        set_sensor_error1(true);
        return false;
    }
    return true;
}

int Lps22hb::begin()
{
    int err = E_NO_ERROR;

    if (init_done) return err;

    err = reset();
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    if(!is_device_id_valid()) return E_NO_DEVICE;

    err = set_interrupt1_handler();
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Enable block data update.
    err = lps22hb_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Enable auto increment.
    err = lps22hb_auto_add_inc_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set fifo watermark to 16 samples.
    err = lps22hb_fifo_watermark_set(dev_ctx, 25);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set fifo mode to dynamic stream.
    err = lps22hb_fifo_mode_set(dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Enable fifo.
    err = lps22hb_fifo_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Enable interrupt when data stored in fifo reaches watermark (threshold) level.
    err = lps22hb_fifo_threshold_on_int_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Put data ready or fifo flags on the interrupt pin.
    err = lps22hb_int_pin_mode_set(dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set interrupt pin mode to push pull.
    err = lps22hb_pin_mode_set(dev_ctx, LPS22HB_PUSH_PULL);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set interrupt pin polarity to active low.
    err = lps22hb_int_polarity_set(dev_ctx, LPS22HB_ACTIVE_LOW);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set low power.
    err = lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set low pass filter.
    err = lps22hb_low_pass_filter_mode_set(dev_ctx, LPS22HB_LPF_ODR_DIV_2);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    // Set Output Data Rate.
    err = lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_25_Hz);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    set_sensor_error1(false);
    return err;
}

int Lps22hb::end()
{
    int err = E_NO_ERROR;

    reset();

    if(!is_device_id_valid()) return E_NO_DEVICE;

    // Set low power.
    err = lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    set_sensor_error1(false);
    init_done = false;
    return err;
}

int Lps22hb::handle_interrupt1()
{
    int err = E_NO_ERROR;

    uint8_t data_level = 0;
    err = lps22hb_fifo_data_level_get(dev_ctx, &data_level);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    err = lps22hb_fifo_output_data_burst_get(dev_ctx, fifo_buffer, data_level);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
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
    avg_temperature += modulo_avg_temperature / data_level;
    data_processor->set_baro_data(avg_pressure, avg_temperature);

    set_sensor_error1(false);
    return err;
}

int Lps22hb::handle_interrupt2()
{

}