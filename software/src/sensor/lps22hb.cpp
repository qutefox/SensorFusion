#include "lps22hb.h"

#include "src/debug_print.h"

void lps22hb_interrupt_callback(void* this_obj)
{
    sensor::Lps22hb* lps22hb = reinterpret_cast<sensor::Lps22hb*>(this_obj);
    debug_print("new lps22hb data\n");
    lps22hb->dump_new_data();
}

sensor::Lps22hb::Lps22hb(io::i2c::I2cMaster* i2c_master, uint8_t i2c_address, mxc_gpio_regs_t *pin_port, uint32_t pin_mask, bool debug)
    : i2c_device{ i2c_master, i2c_address, debug }
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
    
    gpio.port = pin_port;
    gpio.mask = pin_mask;
    gpio.pad = MXC_GPIO_PAD_NONE;
    gpio.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio);
    enable_interrupt_on_gpio();
}

int sensor::Lps22hb::enable_interrupt_on_gpio()
{
    MXC_GPIO_RegisterCallback(&gpio, lps22hb_interrupt_callback, this);
    int err = MXC_GPIO_IntConfig(&gpio, MXC_GPIO_INT_FALLING);
    if (err != E_NO_ERROR) return err;
    MXC_GPIO_EnableInt(gpio.port, gpio.mask);
    NVIC_EnableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
    return err;
}

void sensor::Lps22hb::disable_interrupt_on_gpio()
{
    MXC_GPIO_DisableInt(gpio.port, gpio.mask);
    NVIC_DisableIRQ((IRQn_Type) MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gpio.port)));
}

int sensor::Lps22hb::begin()
{
    // Check device ID.
    uint8_t whoami;
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

    // Enable block data update.
    lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set fifo watermark to 16 samples.
    lps22hb_fifo_watermark_set(&dev_ctx, 16);

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
    lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_1_Hz);

    return E_NO_ERROR;
}

int sensor::Lps22hb::end()
{
    return E_NO_ERROR;
}

void sensor::Lps22hb::dump_new_data()
{
    uint8_t data_level = 0;
    lps22hb_fifo_data_level_get(&dev_ctx, &data_level);

    debug_print("Fifo data level: %d.\n", data_level);

    lps22hb_fifo_output_data_burst_get(&dev_ctx, fifo_buffer, data_level);
    
    for (uint8_t i = 0 ; i < data_level ; ++i)
    {
        int32_t pressure = lps22hb_fifo_output_data_to_raw_pressure(&fifo_buffer[i]);
        int16_t temperature = lps22hb_fifo_output_data_to_raw_temperature(&fifo_buffer[i]);

        float pressure_hpa = lps22hb_from_lsb_to_hpa(pressure);
        float temperature_c = lps22hb_from_lsb_to_degc(temperature);

        debug_print("%d.) pressure: %f hPa, temperature: %f C\n", i, pressure_hpa, temperature_c);
    }
    
}