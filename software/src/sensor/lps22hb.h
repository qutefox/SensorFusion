#pragma once

#include "src/sensor/lps22hb-pid/lps22hb_reg.h"
#include "src/io/i2c/i2c_device.h"
#include "src/io/pin/input_pin.h"

namespace sensor
{

class Lps22hb
{
    stmdev_ctx_t dev_ctx;
    io::i2c::I2cDevice i2c_device;
    io::pin::Input* input_pin = nullptr;
    lps22hb_fifo_output_data_t fifo_buffer[32];

public:
    Lps22hb(io::i2c::I2cMaster* i2c_master, uint8_t i2c_address, io::pin::Input* input_pin, bool debug);

    int begin();

    int end();

    void dump_new_data();
};


} // namespace sensor
