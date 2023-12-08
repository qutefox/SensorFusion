#pragma once

#include <stdint.h>

#include "src/sensor/sensor_types.h"
#include "src/sensor/sensor_base.h"

namespace sensor
{

class Lis2mdl : public SensorBase
{
private:
    static Lis2mdl* instance;
    static uint32_t lock;
    
    io::DigitalInputPinInterface* interrupt_pin = nullptr;
    axis3bit16_t raw_mag;
    temperature_t raw_temperature;
    float celsius = 0;

    virtual int reset() override;
    int is_device_id_matching();

protected:
    Lis2mdl(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin1=nullptr);
    virtual ~Lis2mdl();

public:
    Lis2mdl(Lis2mdl& other) = delete;
    void operator=(const Lis2mdl& other) = delete;

    static Lis2mdl* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin=nullptr);

    virtual int begin() override;
    virtual int end() override;
    
    virtual int set_power_mode(uint8_t device_index, PowerMode power_mode = PowerMode::POWER_DOWN) override;
    virtual uint16_t get_sample_rate_in_hz(uint8_t device_index) override;
    virtual int handle_interrupt() override;
};

} // namespace sensor
