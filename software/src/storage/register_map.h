#pragma once

#include "src/io/output_pin.h"
#include "src/storage/register_types.h"

namespace storage
{

// I2C slave registers.
class RegisterMap
{

    typedef struct __attribute__((packed))
    {
        register_types::sensor_errors_t sensor_errors;
        register_types::data_ready_t data_ready;
        register_types::red_led_t red_led;
    } register_map_t;

    static constexpr register_map_t write_enable_map = {
        { 0, 0, 0, 0, 0 }, // sensor_errors_t
        { 0, 0, 0, 0, 0 }, // data_ready_t
        { 1, 0 } // red_led_t
    };

    io::pin::Output* red_led;
    register_map_t register_map;

    void update_sensor_errors();

public:
    RegisterMap(io::pin::Output* red_led);

    void reset();

    void set_gyro_error(bool error);
    void set_xl_error(bool error);
    void set_mag_error(bool error);
    void set_baro_error(bool error);

    void set_baro_pressure(int32_t pressure);
    void set_baro_temperature(int16_t temperature);

};

} // namespace storage
