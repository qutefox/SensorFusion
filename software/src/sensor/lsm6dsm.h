#pragma once

#include <stdint.h>

#include "src/sensor/sensor_types.h"
#include "src/sensor/sensor_base.h"

namespace sensor
{

enum Lsm6dsmDevice
{
    LSM6DSM_DEVICE_GYRO = 0,
    LSM6DSM_DEVICE_ACCEL = 1,
};

class Lsm6dsm : public SensorBase
{
private:
    static Lsm6dsm* instance;
    static uint32_t lock;

    bool interrupt1_active = false;
    bool interrupt2_active = false;
    int err1 = E_NO_ERROR;
    int err2 = E_NO_ERROR;
    io::DigitalInputPinInterface* interrupt1_pin = nullptr;
    io::DigitalInputPinInterface* interrupt2_pin = nullptr;
    uint8_t gyroscope_data_ready = 0;
    uint8_t accelerometer_data_ready = 0;
    temperature_t raw_temperature;
    axis3bit16_t raw_gyroscope;
    axis3bit16_t raw_accelerometer;
    timestamp_t raw_timestamp;

    virtual int reset() override;
    int is_device_id_matching();
    int handle_interrupt1();
    int handle_interrupt2();

protected:
    Lsm6dsm(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt1_pin=nullptr, io::DigitalInputPinInterface* interrupt2_pin=nullptr);
    virtual ~Lsm6dsm();

public:
    Lsm6dsm(Lsm6dsm& other) = delete;
    void operator=(const Lsm6dsm& other) = delete;

    static Lsm6dsm* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt1_pin=nullptr, io::DigitalInputPinInterface* interrupt2_pin=nullptr);

    virtual int begin() override;
    virtual int end() override;
    
    virtual int set_power_mode(uint8_t device_index, PowerMode power_mode = PowerMode::POWER_DOWN) override;
    virtual bool has_error() override;
    virtual bool has_interrupt() override;
    virtual int handle_interrupt() override;

    void set_interrupt1_active();
    void set_interrupt2_active();
};

} // namespace sensor
