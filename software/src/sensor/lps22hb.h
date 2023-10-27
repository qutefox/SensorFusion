#pragma once

#include "mxc_device.h"
#include "gpio.h"

#include "lps22hb-pid/lps22hb_reg.h"
#include "src/io/i2c/i2c_device.h"

namespace sensor
{

class Lps22hb
{
    mxc_gpio_cfg_t gpio;
    stmdev_ctx_t dev_ctx;
    io::i2c::I2cDevice i2c_device;
    lps22hb_fifo_output_data_t fifo_buffer[32];

    int32_t i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
    int32_t i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

public:
    Lps22hb(io::i2c::I2cMaster* i2c_master, uint8_t i2c_address, mxc_gpio_regs_t *pin_port, uint32_t pin_mask, bool debug);

    int begin();
    int end();

    void dump_new_data();

    int enable_interrupt_on_gpio();
    void disable_interrupt_on_gpio();

    friend void lps22hb_interrupt_callback(void* this_obj);
};


} // namespace sensor
