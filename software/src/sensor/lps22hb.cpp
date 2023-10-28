#include "lps22hb.h"

#include "mxc_errors.h"

void lps22hb_interrupt_callback(void* this_obj)
{
    sensor::Lps22hb* lps22hb = reinterpret_cast<sensor::Lps22hb*>(this_obj);
    lps22hb->process_fifo_data();
}

namespace sensor
{

Lps22hb::Lps22hb(io::i2c::I2cMaster* i2c_master, uint8_t i2c_address,
    io::pin::Input* _input_pin, storage::RegisterMap* _register_map, bool debug)
    : i2c_device{ i2c_master, i2c_address, debug }
    , input_pin{ _input_pin }
    , register_map{ _register_map }
{
    dev_ctx.handle = &i2c_device;
    dev_ctx.write_reg = [](void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = reinterpret_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->write_bytes(reg, bufp, len);
        };
    dev_ctx.read_reg = [](void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::i2c::I2cDevice* i2c_device = reinterpret_cast<io::i2c::I2cDevice*>(handle);
            return i2c_device->read_bytes(reg, bufp, len);
        };
}

int Lps22hb::begin()
{
    int err = E_NO_ERROR;

    // Check device ID.
    uint8_t whoami = 0;
    err = lps22hb_device_id_get(&dev_ctx, &whoami);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    if (whoami != LPS22HB_ID)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Restore default configuration.
    err = lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    uint8_t rst;
    do {
        err = lps22hb_reset_get(&dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            register_map->set_baro_error(true);
            return err;
        }
    } while (rst);

    if (input_pin != nullptr)
    {
        err = input_pin->attach_interrupt_callback(lps22hb_interrupt_callback, this);
        if (err != E_NO_ERROR)
        {
            register_map->set_baro_error(true);
            return err;
        }
    }

    // Enable block data update.
    err = lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set fifo watermark to 16 samples.
    err = lps22hb_fifo_watermark_set(&dev_ctx, 25);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set fifo mode to dynamic stream.
    err = lps22hb_fifo_mode_set(&dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Enable fifo.
    err = lps22hb_fifo_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Enable interrupt when data stored in fifo reaches watermark (threshold) level.
    err = lps22hb_fifo_threshold_on_int_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Put data ready or fifo flags on the interrupt pin.
    err = lps22hb_int_pin_mode_set(&dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set interrupt pin mode to push pull.
    err = lps22hb_pin_mode_set(&dev_ctx, LPS22HB_PUSH_PULL);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set interrupt pin polarity to active low.
    err = lps22hb_int_polarity_set(&dev_ctx, LPS22HB_ACTIVE_LOW);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set low power.
    err = lps22hb_low_power_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set low pass filter.
    err = lps22hb_low_pass_filter_mode_set(&dev_ctx, LPS22HB_LPF_ODR_DIV_2);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    // Set Output Data Rate.
    // TODO: set LPS22HB_ODR_25_Hz. For dev tests we want to take things slow.
    err = lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_1_Hz);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    register_map->set_baro_error(false);
    return E_NO_ERROR;
}

int Lps22hb::end()
{
    int err = E_NO_ERROR;

    // Restore default configuration.
    err = lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    uint8_t rst;
    do
    {
        err = lps22hb_reset_get(&dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            register_map->set_baro_error(true);
            return err;
        }

    } while (rst);

    // Set low power.
    err = lps22hb_low_power_set(&dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    register_map->set_baro_error(false);
    return err;
}

int Lps22hb::process_fifo_data()
{
    int err = E_NO_ERROR;

    uint8_t data_level = 0;
    err = lps22hb_fifo_data_level_get(&dev_ctx, &data_level);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
        return err;
    }

    err = lps22hb_fifo_output_data_burst_get(&dev_ctx, fifo_buffer, data_level);
    if (err != E_NO_ERROR)
    {
        register_map->set_baro_error(true);
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

    register_map->set_baro_pressure(avg_pressure);
    register_map->set_baro_temperature(avg_temperature);

    register_map->set_baro_error(false);
    return err;
}

} // namespace sensor
