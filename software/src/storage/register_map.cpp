#include "register_map.h"

using namespace storage;

RegisterMap::RegisterMap(io::pin::Output* _red_led)
    : red_led{ _red_led }
{
    reset();
}

void RegisterMap::reset()
{
    uint8_t* regs = reinterpret_cast<uint8_t*>(&register_map);
    regs[0] = 0x00; // sensor_errors_t
    regs[1] = 0x00; // data_ready_t
    regs[2] = 0x00; // red_led_t
}

void RegisterMap::set_gyro_error(bool error)
{
    register_map.sensor_errors.gyro_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_xl_error(bool error)
{
    register_map.sensor_errors.xl_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_mag_error(bool error)
{
    register_map.sensor_errors.mag_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_baro_error(bool error)
{
    register_map.sensor_errors.baro_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::update_sensor_errors()
{
    // TODO: in case of error state flash LED.
}

void RegisterMap::set_baro_pressure(int32_t pressure)
{
    // TODO: store baro pressure.
}

void RegisterMap::set_baro_temperature(int16_t temperature)
{
    // TODO: store baro temperature.
}