#pragma once

#include <stdint.h>

#include "sensor_interface.h"
#include "src/io/i2c_device_interface.h"
#include "src/io/digital_input_pin_interface.h"
#include "src/data_processor_interface.h"

// Forward declare types.
struct _stmdev_ctx_t;
typedef _stmdev_ctx_t stmdev_ctx_t;

namespace sensor
{

class SensorBase : public SensorInterface
{
protected:
    DataProcessorInterface* data_processor = nullptr;
    io::I2cDeviceInterface* i2c_device = nullptr;
    stmdev_ctx_t* dev_ctx = nullptr;
    io::DigitalInputPinInterface* interrupt_pin1 = nullptr;
    io::DigitalInputPinInterface* interrupt_pin2 = nullptr;
    bool interrupt1_active = false;
    bool interrupt2_active = false;

    virtual int reset() = 0;
    virtual bool is_device_id_valid() = 0;
    
    virtual int attach_interrupt1_handler(bool attached) override;
    virtual int attach_interrupt2_handler(bool attached) override;

    virtual int handle_interrupt1() override;
    virtual int handle_interrupt2() override;
    
public:
    SensorBase(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin1=nullptr,
        io::DigitalInputPinInterface* interrupt_pin2=nullptr);
    virtual ~SensorBase();

    virtual int begin() = 0;
    virtual int end() override;

    virtual int set_power_mode(PowerMode power_mode) = 0;

    virtual void set_interrupt_pin1(io::DigitalInputPinInterface* interrupt_pin) override;
    virtual void set_interrupt_pin2(io::DigitalInputPinInterface* interrupt_pin) override;
    
    void set_interrupt1_active();
    void set_interrupt2_active();

    virtual bool has_interrupt1() override;
    virtual bool has_interrupt2() override;

    virtual int handle_possible_interrupt() override;
};

} // namespace sensor
