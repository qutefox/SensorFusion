#pragma once

#include <stdint.h>

#include "sensor_base.h"

namespace sensor
{

class Lsm6dsm : public SensorBase
{
private:
    static Lsm6dsm* instance;
    static uint32_t lock;


    virtual int reset() override;
    virtual inline void set_sensor_error1(bool value) override;
    virtual inline void set_sensor_error2(bool value) override;
    virtual bool is_device_id_valid() override;

protected:
    Lsm6dsm(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr, io::pin::Input* interrupt_pin2=nullptr);
    virtual ~Lsm6dsm();

public:
    Lsm6dsm(Lsm6dsm& other) = delete;
    void operator=(const Lsm6dsm& other) = delete;

    static Lsm6dsm* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr, io::pin::Input* interrupt_pin2=nullptr);

    virtual int begin() override;
    virtual int end() override;
    virtual int handle_interrupt1() override;
    virtual int handle_interrupt2() override;
};

} // namespace sensor
