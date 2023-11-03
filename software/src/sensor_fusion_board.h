#pragma once

#include "src/io/input_pin.h"
#include "src/io/output_pin.h"
#include "src/io/i2c_slave.h"

// #include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/sensor/lps22hb.h"
// #include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"

#define CONSOLE_UART 1 // UART instance to use for console
#define CONSOLE_BAUD 115200 // Console baud rate

#define I2C_SLAVE 1
#define I2C_SLAVE_SPEED 400000 // 400 kbit/s (aka i2c fast speed)
#define I2C_SLAVE_ADDR 0x51

#define I2C_MASTER 0
#define I2C_MASTER_SPEED 400000 // 400 kbit/s (aka i2c fast speed)
#define I2C_MASTER_CLOCK_STRETCHING 0

#define LPS22HB_I2C_ADDR 0x5C

#define LED_PIN_MASK MXC_GPIO_PIN_12

#define BARO_INT_MASK MXC_GPIO_PIN_4

class SensorFusionBoard
{
private:
    bool init_done = false;
    static SensorFusionBoard* instance;
    static uint32_t lock;

    io::pin::Output* led_pin = nullptr;
    io::pin::Input* baro_int_pin = nullptr;
    io::i2c::I2cSlave* i2c_slave = nullptr;

    sensor::Lps22hb* lps22hb_sensor = nullptr;

protected:
    SensorFusionBoard();
    virtual ~SensorFusionBoard();

public:
    SensorFusionBoard(SensorFusionBoard& other) = delete;
    void operator=(const SensorFusionBoard& other) = delete;

    static SensorFusionBoard* get_instance();

    int begin();

    io::pin::Output* get_led_pin() const { return led_pin; }
    io::pin::Input* get_baro_int_pin() const { return baro_int_pin; }
    io::i2c::I2cSlave* get_i2c_slave() const { return i2c_slave; }
    sensor::Lps22hb* get_lps22hb_sensor() const { return lps22hb_sensor; }

    void prep_for_sleep();
};

