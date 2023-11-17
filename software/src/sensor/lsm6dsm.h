#pragma once

#include <stdint.h>

#include "src/sensor/sensor_types.h"
#include "src/sensor/sensor_base.h"

namespace sensor
{

class Lsm6dsm : public SensorBase
{
private:
    static Lsm6dsm* instance;
    static uint32_t lock;

    uint8_t gyroscope_data_ready = 0;
    uint8_t accelerometer_data_ready = 0;
    temperature_t raw_temperature;
    axis3bit16_t raw_gyroscope;
    axis3bit16_t raw_accelerometer;
    timestamp_t raw_timestamp;

    virtual int reset() override;
    virtual bool is_device_id_valid() override;

protected:
    Lsm6dsm(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin1=nullptr, io::DigitalInputPinInterface* interrupt_pin2=nullptr);
    virtual ~Lsm6dsm();

    virtual inline int handle_interrupt1() override;
    virtual inline int handle_interrupt2() override;

public:
    Lsm6dsm(Lsm6dsm& other) = delete;
    void operator=(const Lsm6dsm& other) = delete;

    static Lsm6dsm* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin1=nullptr, io::DigitalInputPinInterface* interrupt_pin2=nullptr);

    virtual int begin() override;
    virtual int end() override;
    virtual int set_power_mode(PowerMode power_mode) override;
};

} // namespace sensor
