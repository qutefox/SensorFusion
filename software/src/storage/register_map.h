#pragma once

#include <stdint.h>

#include "src/io/output_pin.h"
#include "src/storage/register_map_type.h"

namespace storage
{

class RegisterMapReaderWriter; // Forward declare.

// I2C slave registers.
class RegisterMap
{
    friend class RegisterMapReaderWriter;

private:
    bool init_done = false;
    static RegisterMap* instance;
    static uint32_t lock;

    io::pin::Output* red_led = nullptr;

    void update_sensor_errors();

protected:
    RegisterMap();
    ~RegisterMap();

public:
    RegisterMap(RegisterMap& other) = delete;
    void operator=(const RegisterMap& other) = delete;

    static RegisterMap* get_instance();

    int begin();
    void set_red_led_instance(io::pin::Output* red_led);

    void set_gyro_error(bool error);
    void set_xl_error(bool error);
    void set_mag_error(bool error);
    void set_baro_error(bool error);

    void set_baro_pressure(int32_t pressure);
    void set_baro_temperature(int16_t temperature);

};

} // namespace storage
