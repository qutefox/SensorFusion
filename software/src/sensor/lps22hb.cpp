#include "lps22hb.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/data_processor.h"
#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

void lps22hb_interrupt_callback(void* this_obj)
{
    // sensor::Lps22hb* lps22hb = sensor::Lps22hb::get_instance();
    sensor::Lps22hb* lps22hb = static_cast<sensor::Lps22hb*>(this_obj);
    lps22hb->process_fifo_data();
}

namespace sensor
{

Lps22hb* Lps22hb::instance = nullptr;
uint32_t Lps22hb::lock = 0;

Lps22hb::Lps22hb()
    : init_done{ false }
    , dev_ctx{ new stmdev_ctx_t() }
    , fifo_buffer{ new lps22hb_fifo_output_data_t[32] }
    , interrupt_pin{ nullptr }
    , i2c_device{ nullptr }
    , data_processor{ DataProcessor::get_instance() }
{

}

Lps22hb::~Lps22hb()
{
    if (init_done)
    {
        end();
        delete i2c_device;
    }
    delete dev_ctx;
    delete[] fifo_buffer;
    init_done = false;
}

Lps22hb* Lps22hb::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lps22hb();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Lps22hb::begin(uint8_t i2c_address, bool debug, io::pin::Input* interrupt_pin)
{
    int err = E_NO_ERROR;

    if (init_done) return err;

    i2c_device = new io::i2c::I2cDevice(i2c_address, debug);
    err = i2c_device->begin();
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    dev_ctx->handle = i2c_device;
    dev_ctx->write_reg =
        [](void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = static_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->write_bytes(reg, bufp, len);
        };
    dev_ctx->read_reg =
        [](void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = static_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->read_bytes(reg, bufp, len);
        };

    // Check device ID.
    uint8_t whoami = 0;
    err = lps22hb_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    if (whoami != LPS22HB_ID)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Restore default configuration.
    err = lps22hb_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Wait for sensor to complete software reset.
    uint8_t rst;
    do
    {
        err = lps22hb_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            data_processor->set_baro_sensor_error(true);
            return err;
        }
    }
    while(rst);

    // Attach interrupt handler.
    if (interrupt_pin != nullptr)
    {
        err = interrupt_pin->attach_interrupt_callback(lps22hb_interrupt_callback, this);
        if (err != E_NO_ERROR)
        {
            data_processor->set_baro_sensor_error(true);
            return err;
        }

        interrupt_pin->set_wake_up_enable(true);
    }

    // Enable block data update.
    err = lps22hb_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set fifo watermark to 16 samples.
    err = lps22hb_fifo_watermark_set(dev_ctx, 25);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set fifo mode to dynamic stream.
    err = lps22hb_fifo_mode_set(dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Enable fifo.
    err = lps22hb_fifo_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Enable interrupt when data stored in fifo reaches watermark (threshold) level.
    err = lps22hb_fifo_threshold_on_int_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Put data ready or fifo flags on the interrupt pin.
    err = lps22hb_int_pin_mode_set(dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set interrupt pin mode to push pull.
    err = lps22hb_pin_mode_set(dev_ctx, LPS22HB_PUSH_PULL);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set interrupt pin polarity to active low.
    err = lps22hb_int_polarity_set(dev_ctx, LPS22HB_ACTIVE_LOW);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set low power.
    err = lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set low pass filter.
    err = lps22hb_low_pass_filter_mode_set(dev_ctx, LPS22HB_LPF_ODR_DIV_2);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Set Output Data Rate.
    err = lps22hb_data_rate_set(dev_ctx, LPS22HB_ODR_25_Hz);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    data_processor->set_baro_sensor_error(false);
    return err;
}

int Lps22hb::end()
{
    int err = E_NO_ERROR;

    // Restore default configuration.
    err = lps22hb_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Wait for sensor to complete software reset.
    uint8_t rst;
    do
    {
        err = lps22hb_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            data_processor->set_baro_sensor_error(true);
            return err;
        }

    }
    while (rst);

    // Set low power.
    err = lps22hb_low_power_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    // Detach interrupt handler.
    if (interrupt_pin != nullptr)
    {
        interrupt_pin->detach_interrupt_callback();
    }

    data_processor->set_baro_sensor_error(false);
    return err;
}

int Lps22hb::process_fifo_data()
{
    int err = E_NO_ERROR;

    uint8_t data_level = 0;
    err = lps22hb_fifo_data_level_get(dev_ctx, &data_level);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
        return err;
    }

    err = lps22hb_fifo_output_data_burst_get(dev_ctx, fifo_buffer, data_level);
    if (err != E_NO_ERROR)
    {
        data_processor->set_baro_sensor_error(true);
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

    data_processor->set_baro_sensor_error(false);
    return err;
}

} // namespace sensor
