#pragma once

#include "src/io/output_pin.h"
#include "src/storage/register_types.h"

namespace storage
{

typedef struct __attribute__((packed))
{
    register_types::sensor_errors_t sensor_errors;
    register_types::data_ready_t data_ready;
    register_types::red_led_t red_led;
    uint8_t baro_pressure[3];
    int16_t baro_temp;
} register_map_t;

// I2C slave registers.
class RegisterMap
{

private:
    bool init_done = false;
    static RegisterMap* instance;
    static uint32_t lock;

    io::pin::Output* red_led;
    register_map_t register_map;

    void update_sensor_errors();

protected:
    RegisterMap();
    ~RegisterMap();

public:
    RegisterMap(RegisterMap& other) = delete;
    void operator=(const RegisterMap& other) = delete;

    static RegisterMap* get_instance();

    int begin();

    void reset();

    bool is_address_in_range(uint8_t addr);
    uint8_t read(uint8_t addr);
    void write(uint8_t addr, uint8_t value);

    void set_gyro_error(bool error);
    void set_xl_error(bool error);
    void set_mag_error(bool error);
    void set_baro_error(bool error);

    void set_baro_pressure(int32_t pressure);
    void set_baro_temperature(int16_t temperature);

};

} // namespace storage
