#include "lps22hb.h"

#include "mxc_errors.h"

#include "src/debug_print.h"

void lps22hb_interrupt_callback(void* this_obj)
{
    sensor::Lps22hb* lps22hb = reinterpret_cast<sensor::Lps22hb*>(this_obj);
    lps22hb->process_fifo_data();
}

namespace sensor
{

Lps22hb::Lps22hb(io::i2c::I2cMaster* i2c_master, uint8_t i2c_address, io::pin::Input* _input_pin, bool debug)
    : i2c_device{ i2c_master, i2c_address, debug }
    , input_pin{ _input_pin }
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

// TODO: useful return value.
// TODO: set in_error flag somewhere.
int Lps22hb::begin()
{
    // Check device ID.
    uint8_t whoami = 0;
    lps22hb_device_id_get(&dev_ctx, &whoami);
    if (whoami != LPS22HB_ID)
    {
        debug_print("LPS22HB wrong device id: %02X.\n", whoami);
    }

    // Restore default configuration.
    lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lps22hb_reset_get(&dev_ctx, &rst);
    } while (rst);

    if (input_pin != nullptr)
    {
        input_pin->attach_interrupt_callback(lps22hb_interrupt_callback, this);
    }

    // Enable block data update.
    lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set fifo watermark to 16 samples.
    lps22hb_fifo_watermark_set(&dev_ctx, 25);

    // Set fifo mode to dynamic stream.
    lps22hb_fifo_mode_set(&dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);

    // Enable fifo.
    lps22hb_fifo_set(&dev_ctx, PROPERTY_ENABLE);

    // Enable interrupt when data stored in fifo reaches watermark (threshold) level.
    lps22hb_fifo_threshold_on_int_set(&dev_ctx, PROPERTY_ENABLE);

    // Put data ready or fifo flags on the interrupt pin.
    lps22hb_int_pin_mode_set(&dev_ctx, LPS22HB_DRDY_OR_FIFO_FLAGS);

    // Set interrupt pin mode to push pull.
    lps22hb_pin_mode_set(&dev_ctx, LPS22HB_PUSH_PULL);

    // Set interrupt pin polarity to active low.
    lps22hb_int_polarity_set(&dev_ctx, LPS22HB_ACTIVE_LOW);

    // Set low power.
    lps22hb_low_power_set(&dev_ctx, PROPERTY_ENABLE);

    // Set low pass filter.
    lps22hb_low_pass_filter_mode_set(&dev_ctx, LPS22HB_LPF_ODR_DIV_2);

    // Set Output Data Rate.
    // TODO: set LPS22HB_ODR_25_Hz. For dev tests we want to take things slow.
    lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_1_Hz);

    return E_NO_ERROR;
}

void Lps22hb::end()
{
    // Restore default configuration.
    lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lps22hb_reset_get(&dev_ctx, &rst);
    } while (rst);

    // Set low power.
    lps22hb_low_power_set(&dev_ctx, PROPERTY_ENABLE);
}

// TODO: set in_error flag somewhere (on error).
void Lps22hb::process_fifo_data()
{
    uint8_t data_level = 0;
    lps22hb_fifo_data_level_get(&dev_ctx, &data_level);

    lps22hb_fifo_output_data_burst_get(&dev_ctx, fifo_buffer, data_level);

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

    // TODO: store/update current baro data in memory somewhere so host mcu can query it.

    // For the time being we just dump it to the serial debug port.
    float pressure_hpa = lps22hb_from_lsb_to_hpa(avg_pressure);
    float temperature_c = lps22hb_from_lsb_to_degc(avg_temperature);

    debug_print("avg.) pressure: %f hPa, temperature: %f C\n", pressure_hpa, temperature_c);
}

} // namespace sensor
