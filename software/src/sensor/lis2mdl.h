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
    
    axis3bit16_t raw_mag;
    temperature_t raw_temperature;

    virtual int reset() override;
    virtual inline void set_sensor_error1(bool value) override;
    virtual bool is_device_id_valid() override;

protected:
    Lis2mdl(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr);
    virtual ~Lis2mdl();

public:
    Lis2mdl(Lis2mdl& other) = delete;
    void operator=(const Lis2mdl& other) = delete;

    static Lis2mdl* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr);

    virtual int begin() override;
    virtual int end() override;
    virtual int set_power_mode(PowerMode power_mode) override;
    virtual inline int handle_interrupt1() override;
};

} // namespace sensor
