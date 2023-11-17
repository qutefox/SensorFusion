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
    : led_pin{ nullptr }
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

    MXC_Delay(MXC_DELAY_MSEC(100));

    led_pin = new io::DigitalOutputPin(MXC_GPIO0, LED_PIN_MASK);
    led_pin->write(true);

#ifndef ENABLE_DEBUG_PRINT
    host_interrupt_pin = new io::DigitalOutputPin(MXC_GPIO0, HOSTINT_PIN_MASK);
    host_interrupt_pin->write(true); // Active low, push pull.
#endif

    i2c_slave_address_select_pin = new io::DigitalInputPin(MXC_GPIO0, AD0_MASK__SWDCLK_MASK, mxc_gpio_pad_t::MXC_GPIO_PAD_PULL_DOWN);

    barometer_int_pin = new io::DigitalInputPin(MXC_GPIO0, BARO_INT_MASK);
    inertial_int1_pin = new io::DigitalInputPin(MXC_GPIO0, INERTIAL_INT1_MASK);
    inertial_int2_pin = new io::DigitalInputPin(MXC_GPIO0, INERTIAL_INT2_MASK);
    magnetometer_int_pin = new io::DigitalInputPin(MXC_GPIO0, MAG_INT_MASK);

    barometer_sensor = sensor::Lps22hb::get_instance(LPS22HB_I2C_ADDR, false, barometer_int_pin);
    err |= barometer_sensor->begin();
    barometer_sensor->set_power_mode(sensor::PowerMode::LOW_POWER);

    inertial_sensor = sensor::Lsm6dsm::get_instance(LSM6DSM_I2C_ADDR, false, inertial_int1_pin, inertial_int2_pin);
    err |= inertial_sensor->begin();

    magnetometer_sensor = sensor::Lis2mdl::get_instance(LIS2MDL_I2C_ADDR, false, magnetometer_int_pin);
    err |= magnetometer_sensor->begin();

    bool ad0_input_pin_value = 0;
    err |= i2c_slave_address_select_pin->read(ad0_input_pin_value);
    i2c_slave_address_select_pin->set_pullup_pulldown(mxc_gpio_pad_t::MXC_GPIO_PAD_NONE);

    i2c_slave = io::I2cSlave::get_instance();
    err |= i2c_slave->begin(I2C_SLAVE_ADDR + (ad0_input_pin_value ? 1 : 0));

    inertial_sensor->set_power_mode(sensor::PowerMode::LOW_POWER);
    barometer_sensor->set_power_mode(sensor::PowerMode::LOW_POWER);

    led_pin->write(false);

    return err;
}

void SensorFusionBoard::prepare_for_sleep()
{
#ifdef ENABLE_DEBUG_PRINT
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif
}
