#pragma once

#include <stdint.h>

#include "mxc_errors.h"

#include "sensor_interface.h"
#include "src/io/i2c_device_interface.h"
#include "src/processor_logic/data_processor_interface.h"

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
    bool interrupt_active = false;
    int err = E_NO_ERROR;

    virtual int reset() = 0;
    
public:
    SensorBase(uint8_t i2c_address, bool i2c_debug=false);
    virtual ~SensorBase();

    virtual int begin() = 0;
    virtual int end() override;

    virtual int set_power_mode(uint8_t device_index, PowerMode power_mode = PowerMode::POWER_DOWN) = 0;
    virtual bool has_error() override;
    virtual bool has_interrupt() override;
    virtual int handle_interrupt() = 0;

    void set_interrupt_active();

};

} // namespace sensor
