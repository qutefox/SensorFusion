#pragma once

#include "src/io/input_pin.h"
#include "src/io/output_pin.h"
#include "src/io/i2c_slave.h"

#include "src/sensor/sensor_interface.h"

#define CONSOLE_UART 1 // UART instance to use for console
#define CONSOLE_BAUD 115200 // Console baud rate

#define I2C_SLAVE 1
#define I2C_SLAVE_SPEED 400000 // 400 kbit/s (aka i2c fast speed)
#define I2C_SLAVE_ADDR 0x51

#define I2C_MASTER 0
#define I2C_MASTER_SPEED 400000 // 400 kbit/s (aka i2c fast speed)
#define I2C_MASTER_CLOCK_STRETCHING 0

#define LPS22HB_I2C_ADDR 0x5C
#define LSM6DSM_I2C_ADDR 0x6A

#define BARO_INT_MASK MXC_GPIO_PIN_4
#define MAG_INT_MASK MXC_GPIO_PIN_5
#define GYRO_INT_MASK MXC_GPIO_PIN_6
#define ACCEL_INT_MASK MXC_GPIO_PIN_7
#define HOSTINT_PIN_MASK MXC_GPIO_PIN_10
#define LED_PIN_MASK MXC_GPIO_PIN_12
#define WAKE_PIN_MASK MXC_GPIO_PIN_13

class SensorFusionBoard
{
private:
    bool init_done = false;
    static SensorFusionBoard* instance;
    static uint32_t lock;

    io::pin::Output* led_pin = nullptr;
    io::pin::Input* baro_int_pin = nullptr;
    io::pin::Input* mag_int_pin = nullptr;
    io::pin::Input* gyro_int_pin = nullptr;
    io::pin::Input* accel_int_pin = nullptr;
    io::i2c::I2cSlave* i2c_slave = nullptr;
    sensor::SensorInterface* lps22hb_sensor = nullptr;
    sensor::SensorInterface* lsm6dsm_sensor = nullptr;
    sensor::SensorInterface* lis2mdl_sensor = nullptr;

protected:
    SensorFusionBoard();
    virtual ~SensorFusionBoard();

public:
    SensorFusionBoard(SensorFusionBoard& other) = delete;
    void operator=(const SensorFusionBoard& other) = delete;

    static SensorFusionBoard* get_instance();

    int begin();

    io::pin::Output* get_led_pin() const { return led_pin; }

    void prep_for_sleep();
};

