#pragma once

#include <stdint.h>

#include "src/sensor/sensor_types.h"
#include "src/sensor/sensor_base.h"

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

    pressure_t raw_pressure;
    temperature_t raw_temperature;

    lps22hb_fifo_output_data_t* fifo_buffer = nullptr;

    virtual int reset() override;
    virtual inline void set_sensor_error1(bool value) override;
    virtual bool is_device_id_valid() override;

protected:
    Lps22hb(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin=nullptr);
    virtual ~Lps22hb();

public:
    Lps22hb(Lps22hb& other) = delete;
    void operator=(const Lps22hb& other) = delete;

    static Lps22hb* get_instance(uint8_t i2c_address, bool i2c_debug=false,
        io::pin::Input* interrupt_pin=nullptr);

    virtual int begin() override;
    virtual int end() override;
    virtual int set_power_mode(PowerMode power_mode) override;
    virtual inline int handle_interrupt1() override;
};


} // namespace sensor
