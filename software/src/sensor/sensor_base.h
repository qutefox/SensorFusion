#pragma once

#include <stdint.h>

#include "sensor_interface.h"
#include "src/io/i2c_device.h"
#include "src/data_processor_interface.h"

// Forward declare types.
struct _stmdev_ctx_t;
typedef _stmdev_ctx_t stmdev_ctx_t;

namespace sensor
{

class SensorBase : public SensorInterface
{
protected:
    bool init_done = false;
    DataProcessorInterface* data_processor = nullptr;
    io::i2c::I2cDevice* i2c_device = nullptr;
    stmdev_ctx_t* dev_ctx = nullptr;
    io::pin::Input* interrupt_pin1 = nullptr;
    io::pin::Input* interrupt_pin2 = nullptr;

    virtual int reset() = 0;
    virtual inline void set_sensor_error1(bool value) = 0;
    virtual inline void set_sensor_error2(bool value) = 0;
    virtual bool is_device_id_valid() = 0;
    virtual int set_interrupt1_handler() override;
    virtual int set_interrupt2_handler() override;

public:
    SensorBase(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin1=nullptr, io::pin::Input* interrupt_pin2=nullptr);
    virtual ~SensorBase();

    virtual int begin() = 0;
    virtual int end() override;

    virtual void set_interrupt_pin1(io::pin::Input* interrupt_pin) override;
    virtual void set_interrupt_pin2(io::pin::Input* interrupt_pin) override;
    
    virtual int handle_interrupt1() override;
    virtual int handle_interrupt2() override;
};

} // namespace sensor
