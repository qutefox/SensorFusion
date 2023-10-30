#pragma once

#include "src/io/i2c_device.h"
#include "src/io/input_pin.h"

// I do not wish to include "src/sensor/lps22hb-pid/lps22hb_reg.h" here.
// I would like to only forward declare the absolute minimum stuff.

// Forward declare types.
struct _stmdev_ctx_t;
typedef _stmdev_ctx_t stmdev_ctx_t;

struct _lps22hb_fifo_output_data_t;
typedef _lps22hb_fifo_output_data_t lps22hb_fifo_output_data_t;

// Since we are using pointers to these types in this header file
// it is enough for the compiler to know that these exist and the
// size of them can be unknown.


class DataProcessor; // Forward declare.

namespace sensor
{

class Lps22hb
{
private:
    bool init_done = false;
    static Lps22hb* instance;
    static uint32_t lock;

    stmdev_ctx_t* dev_ctx = nullptr;
    lps22hb_fifo_output_data_t* fifo_buffer = nullptr;
    io::pin::Input* interrupt_pin = nullptr;
    io::i2c::I2cDevice* i2c_device = nullptr;
    DataProcessor* data_processor;

protected:
    Lps22hb();
    ~Lps22hb();

public:
    Lps22hb(Lps22hb& other) = delete;
    void operator=(const Lps22hb& other) = delete;

    static Lps22hb* get_instance();

    int begin(uint8_t i2c_address, bool debug= false, io::pin::Input* interrupt_pin = nullptr);

    int end();

    int process_fifo_data();
};


} // namespace sensor
