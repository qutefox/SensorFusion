#include "lsm6dsm.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/data_processor.h"
#include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"

using namespace sensor;

Lsm6dsm* Lsm6dsm::instance = nullptr;
uint32_t Lsm6dsm::lock = 0;

void lsm6dsm_interrupt1_callback(void* this_obj)
{
    sensor::Lsm6dsm* lsm6dsm = static_cast<sensor::Lsm6dsm*>(this_obj);
    lsm6dsm->handle_interrupt1();
}

void lsm6dsm_interrupt2_callback(void* this_obj)
{
    sensor::Lsm6dsm* lsm6dsm = static_cast<sensor::Lsm6dsm*>(this_obj);
    lsm6dsm->handle_interrupt2();
}

Lsm6dsm::Lsm6dsm(uint8_t i2c_address, bool i2c_debug, io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
    : SensorBase(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2)
{

}

Lsm6dsm::~Lsm6dsm()
{
    
}

Lsm6dsm* Lsm6dsm::get_instance(uint8_t i2c_address, bool i2c_debug,
    io::pin::Input* interrupt_pin1, io::pin::Input* interrupt_pin2)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Lsm6dsm(i2c_address, i2c_debug, interrupt_pin1, interrupt_pin2);
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Lsm6dsm::reset()
{
    int err = lsm6dsm_reset_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    uint8_t rst;
    do
    {
        err = lsm6dsm_reset_get(dev_ctx, &rst);
        if (err != E_NO_ERROR)
        {
            set_sensor_error1(true);
            set_sensor_error2(true);
            return err;
        }
    }
    while(rst);
}

void Lsm6dsm::set_sensor_error1(bool value)
{
    data_processor->set_gyro_sensor_error(value);
}

void Lsm6dsm::set_sensor_error2(bool value)
{
    data_processor->set_accel_sensor_error(value);
}

bool Lsm6dsm::is_device_id_valid()
{
    uint8_t whoami = 0;
    int err = lsm6dsm_device_id_get(dev_ctx, &whoami);
    if (err != E_NO_ERROR || whoami != LSM6DSM_ID)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return false;
    }
    return true;
}

int Lsm6dsm::begin()
{
    int err = E_NO_ERROR;

    if (init_done) return err;

    reset();

    if(!is_device_id_valid()) return E_NO_DEVICE;

    err = set_interrupt1_handler();
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        return err;
    }

    err = set_interrupt2_handler();
    if (err != E_NO_ERROR)
    {
        set_sensor_error2(true);
        return err;
    }

    // Enable block data update.
    err = lsm6dsm_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    // Enable auto increment.
    err = lsm6dsm_auto_increment_set(dev_ctx, PROPERTY_ENABLE);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    // Set gyro output data rate
    lsm6dsm_gy_data_rate_set(dev_ctx, LSM6DSM_GY_ODR_52Hz);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
    }

    // Set accel output data rate
    lsm6dsm_xl_data_rate_set(dev_ctx, LSM6DSM_XL_ODR_52Hz);
    if (err != E_NO_ERROR)
    {
        set_sensor_error2(true);
    }

    // Set interrupt pin mode to push pull.
    err = lsm6dsm_pin_mode_set(dev_ctx, LSM6DSM_PUSH_PULL);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    // Set interrupt pin polarity to active low.
    err = lsm6dsm_pin_polarity_set(dev_ctx, LSM6DSM_ACTIVE_LOW);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    err = lsm6dsm_xl_lp2_bandwidth_set(dev_ctx, LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_9);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    err = lsm6dsm_data_ready_mode_set(dev_ctx, LSM6DSM_DRDY_LATCHED);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    lsm6dsm_int1_route_t int1_reg;
    int1_reg.int1_sign_mot = 1;
    err = lsm6dsm_pin_int1_route_set(dev_ctx, int1_reg);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    lsm6dsm_int2_route_t int2_reg;
    int2_reg.int2_drdy_g = 1;
    int2_reg.int2_drdy_xl = 1;
    err = lsm6dsm_pin_int2_route_set(dev_ctx, int2_reg);
    if (err != E_NO_ERROR)
    {
        set_sensor_error1(true);
        set_sensor_error2(true);
        return err;
    }

    set_sensor_error1(false);
    set_sensor_error2(false);
    return err;
}

int Lsm6dsm::end()
{
    int err = E_NO_ERROR;

    reset();

    if(!is_device_id_valid()) return E_NO_DEVICE;

    // TODO: set low power if possible.

    set_sensor_error1(false);
    set_sensor_error2(false);
    init_done = false;
    return err;
}

int Lsm6dsm::handle_interrupt1()
{
    int err = E_NO_ERROR;

    // TODO: 

    set_sensor_error1(false);
    return err;
}

int Lsm6dsm::handle_interrupt2()
{
    int err = E_NO_ERROR;

    // TODO: 

    set_sensor_error2(false);
    return err;
}
