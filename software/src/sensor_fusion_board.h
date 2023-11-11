#pragma once

#include "src/io/digital_input_pin_interface.h"
#include "src/io/digital_output_pin_interface.h"
#include "src/io/i2c_slave_interface.h"
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
#define LIS2MDL_I2C_ADDR 0x1E

#define BARO_INT_MASK MXC_GPIO_PIN_4
#define MAG_INT_MASK MXC_GPIO_PIN_5
#define INERTIAL_INT2_MASK MXC_GPIO_PIN_6
#define INERTIAL_INT1_MASK MXC_GPIO_PIN_7
#define HOSTINT_PIN_MASK MXC_GPIO_PIN_10
#define LED_PIN_MASK MXC_GPIO_PIN_12
#define WAKE_PIN_MASK MXC_GPIO_PIN_13

class SensorFusionBoard
{
private:
    static SensorFusionBoard* instance;
    static uint32_t lock;

    io::DigitalOutputPinInterface* led_pin = nullptr;
    io::DigitalInputPinInterface* barometer_int_pin = nullptr;
    io::DigitalInputPinInterface* inertial_int1_pin = nullptr;
    io::DigitalInputPinInterface* inertial_int2_pin = nullptr;
    io::DigitalInputPinInterface* magnetometer_int_pin = nullptr;
    io::I2cSlaveInterface* i2c_slave = nullptr;
    sensor::SensorInterface* barometer_sensor = nullptr;
    sensor::SensorInterface* inertial_sensor = nullptr;
    sensor::SensorInterface* magnetometer_sensor = nullptr;

protected:
    SensorFusionBoard();
    virtual ~SensorFusionBoard();

public:
    SensorFusionBoard(SensorFusionBoard& other) = delete;
    void operator=(const SensorFusionBoard& other) = delete;

    static SensorFusionBoard* get_instance();

    int begin();

    io::DigitalOutputPinInterface* get_led_pin() const { return led_pin; }
    io::I2cSlaveInterface* get_i2c_slave() const { return i2c_slave; }
    sensor::SensorInterface* get_barometer_sensor() const { return barometer_sensor; }
    sensor::SensorInterface* get_inertial_sensor() const { return inertial_sensor; }
    sensor::SensorInterface* get_magnetometer_sensor() const { return magnetometer_sensor; }

    void prep_for_sleep();
};

