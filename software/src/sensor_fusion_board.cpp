#include "sensor_fusion_board.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "uart.h"

#include "src/sensor/lps22hb.h"
#include "src/sensor/lsm6dsm.h"
#include "src/sensor/lis2mdl.h"
#include "src/debug_print.h"

SensorFusionBoard* SensorFusionBoard::instance = nullptr;
uint32_t SensorFusionBoard::lock = 0;

SensorFusionBoard::SensorFusionBoard()
    : init_done{ false }
    , led_pin{ nullptr }
    , baro_int_pin{ nullptr }
    , i2c_slave{ nullptr }
{
    
}

SensorFusionBoard::~SensorFusionBoard()
{
    delete led_pin;
    delete baro_int_pin;
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
    if (init_done) return err;

#ifdef ENABLE_DEBUG_PRINT
        MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), CONSOLE_BAUD, MAP_A);
#endif

    led_pin = new io::pin::Output(MXC_GPIO0, LED_PIN_MASK);
    err |= led_pin->begin();

    baro_int_pin = new io::pin::Input(MXC_GPIO0, BARO_INT_MASK);
    err |= baro_int_pin->begin();

    inertial_int1_pin = new io::pin::Input(MXC_GPIO0, INERTIAL_INT1_MASK);
    err |= inertial_int1_pin->begin();

    inertial_int2_pin = new io::pin::Input(MXC_GPIO0, INERTIAL_INT2_MASK);
    err |= inertial_int2_pin->begin();

    mag_int_pin = new io::pin::Input(MXC_GPIO0, MAG_INT_MASK);
    err |= mag_int_pin->begin();

    i2c_slave = io::i2c::I2cSlave::get_instance();
    err |= i2c_slave->begin();

    baro_sensor = sensor::Lps22hb::get_instance(LPS22HB_I2C_ADDR, false, baro_int_pin);
    err |= baro_sensor->begin();

    inertial_sensor = sensor::Lsm6dsm::get_instance(LSM6DSM_I2C_ADDR, false, inertial_int1_pin, inertial_int2_pin);
    err |= inertial_sensor->begin();

    mag_sensor = sensor::Lis2mdl::get_instance(LIS2MDL_I2C_ADDR, false, mag_int_pin);
    err |= mag_sensor->begin();

    init_done = true;
    return err;
}

void SensorFusionBoard::prep_for_sleep()
{
#ifdef ENABLE_DEBUG_PRINT
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif
}
