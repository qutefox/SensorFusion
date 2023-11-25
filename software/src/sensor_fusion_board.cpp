#include "sensor_fusion_board.h"

#include "mxc_device.h"
#include "nvic_table.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "uart.h"

#include "src/io/digital_input_pin.h"
#include "src/io/digital_output_pin.h"
#include "src/io/i2c_slave.h"
#include "src/sensor/lps22hb.h"
#include "src/sensor/lsm6dsm.h"
#include "src/sensor/lis2mdl.h"
#include "src/debug_print.h"

SensorFusionBoard* SensorFusionBoard::instance = nullptr;
uint32_t SensorFusionBoard::lock = 0;

SensorFusionBoard::SensorFusionBoard()
    : flash_data{ nullptr }
    , led_pin{ nullptr }
    , host_interrupt_pin{ nullptr }
    , i2c_slave_address_select_pin{ nullptr }
    , barometer_int_pin{ nullptr }
    , inertial_int1_pin{ nullptr }
    , inertial_int2_pin{ nullptr }
    , magnetometer_int_pin{ nullptr }
    , i2c_slave{ nullptr }
    , barometer_sensor{ nullptr }
    , inertial_sensor{ nullptr }
    , magnetometer_sensor{ nullptr }
    , i2c_slave_err{ E_NO_ERROR }
{
    
}

SensorFusionBoard::~SensorFusionBoard()
{
    delete led_pin;
    delete barometer_int_pin;
    delete inertial_int1_pin;
    delete inertial_int2_pin;
    delete magnetometer_int_pin;
}

SensorFusionBoard* SensorFusionBoard::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new SensorFusionBoard();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int SensorFusionBoard::begin()
{
    int err = E_NO_ERROR;

    // Initalise flash data storage. Not sure but I think this should go before
    // anything else.
    flash_data = storage::FlashData::get_instance();

    // Initalise LED output pin.
    led_pin = new io::DigitalOutputPin(MXC_GPIO0, LED_PIN_MASK);
    led_pin->write(true);

#ifndef ENABLE_DEBUG_PRINT
    host_interrupt_pin = new io::DigitalOutputPin(MXC_GPIO0, HOSTINT_PIN_MASK);
    host_interrupt_pin->write(true); // Active low, push pull.
#endif

    // Initalise i2c slave address input pin (with internal pulldown).
    i2c_slave_address_select_pin = new io::DigitalInputPin(MXC_GPIO0, AD0_MASK__SWDCLK_MASK, mxc_gpio_pad_t::MXC_GPIO_PAD_PULL_DOWN);

    // Initalise sensor interrupt pins.
    barometer_int_pin = new io::DigitalInputPin(MXC_GPIO0, BARO_INT_MASK);
    inertial_int1_pin = new io::DigitalInputPin(MXC_GPIO0, INERTIAL_INT1_MASK);
    inertial_int2_pin = new io::DigitalInputPin(MXC_GPIO0, INERTIAL_INT2_MASK);
    magnetometer_int_pin = new io::DigitalInputPin(MXC_GPIO0, MAG_INT_MASK);

    // Initalise barometer sensor.
    barometer_sensor = sensor::Lps22hb::get_instance(LPS22HB_I2C_ADDR, false, barometer_int_pin);
    err |= barometer_sensor->begin();

    // Initalise inertial (gyroscope, accelerometer) sensor.
    inertial_sensor = sensor::Lsm6dsm::get_instance(LSM6DSM_I2C_ADDR, false, inertial_int1_pin, inertial_int2_pin);
    err |= inertial_sensor->begin();

    // Initalise magnetometer sensor.
    magnetometer_sensor = sensor::Lis2mdl::get_instance(LIS2MDL_I2C_ADDR, false, magnetometer_int_pin);
    err |= magnetometer_sensor->begin();

    // Read i2c slave address config.
    uint8_t i2c_slave_address = get_i2c_slave_address();
    debug_print("i2c slave address: %d.\n", i2c_slave_address);
    // Remove pulldown from i2c slave address input pin.
    i2c_slave_address_select_pin->set_pullup_pulldown(mxc_gpio_pad_t::MXC_GPIO_PAD_NONE);
    // Initalise i2c slave.
    i2c_slave = io::I2cSlave::get_instance();
    i2c_slave_err = i2c_slave->begin(i2c_slave_address);
    err |= i2c_slave_err;

    led_pin->write(false);

    return err;
}

uint8_t SensorFusionBoard::get_i2c_slave_address()
{
    bool ad0_input_pin_value = 0;
    if (i2c_slave_address_select_pin->read(ad0_input_pin_value) == E_NO_ERROR)
    {
        return I2C_SLAVE_ADDR + (ad0_input_pin_value ? 1 : 0);
    }
    return I2C_SLAVE_ADDR;
}

void SensorFusionBoard::prepare_for_sleep()
{
#ifdef ENABLE_DEBUG_PRINT
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif
}

void SensorFusionBoard::handle_sensor_interrupts()
{
    if (barometer_sensor->has_interrupt())
    {
        barometer_sensor->handle_interrupt();
    }

    if (magnetometer_sensor->has_interrupt())
    {
        magnetometer_sensor->handle_interrupt();
    }

    if (inertial_sensor->has_interrupt())
    {
        inertial_sensor->handle_interrupt();
    }
}