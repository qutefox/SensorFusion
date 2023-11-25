#pragma once

#include <stdint.h>

#include "src/sensor/sensor_types.h"
#include "src/sensor/sensor_base.h"
#include "src/io/digital_input_pin_interface.h"

// Forward declare types.
struct _lps22hb_fifo_output_data_t;
typedef _lps22hb_fifo_output_data_t lps22hb_fifo_output_data_t;

namespace sensor
{

class Lps22hb : public SensorBase
{
private:
    static Lps22hb* instance;
    static uint32_t lock;

    io::DigitalInputPinInterface* interrupt_pin = nullptr;
    uint8_t pressure_data_ready = 0;
    uint8_t temperature_data_ready = 0;
    pressure_t raw_pressure = {0};
    temperature_t raw_temperature = {0};

    virtual int reset() override;
    int is_device_id_matching();

protected:
    Lps22hb(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin=nullptr);
    virtual ~Lps22hb();

public:
    Lps22hb(Lps22hb& other) = delete;
    void operator=(const Lps22hb& other) = delete;

    static Lps22hb* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::DigitalInputPinInterface* interrupt_pin=nullptr);

    virtual int begin() override;
    virtual int end() override;
    
    virtual int set_power_mode(uint8_t device_index, PowerMode power_mode = PowerMode::POWER_DOWN) override;
    virtual int handle_interrupt() override;
};


} // namespace sensor
