#include "register_map.h"

#include "mxc_lock.h"

#include "src/debug_print.h"

namespace storage
{

// TODO: protection againt writing a register value while i2c slave reads it.

RegisterMap* RegisterMap::instance = nullptr;
uint32_t RegisterMap::lock = 0;

RegisterMap::RegisterMap()
    : red_led{ nullptr }
{

}

RegisterMap::~RegisterMap()
{

}

RegisterMap* RegisterMap::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new RegisterMap();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int RegisterMap::begin()
{
    return E_NO_ERROR;
}

void RegisterMap::set_red_led_instance(io::pin::Output* _red_led)
{
    if (_red_led != nullptr) red_led = _red_led;
}

void RegisterMap::set_gyro_error(bool error)
{
    // register_map.sensor_errors.gyro_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_xl_error(bool error)
{
    // register_map.sensor_errors.xl_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_mag_error(bool error)
{
    // register_map.sensor_errors.mag_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::set_baro_error(bool error)
{
    // register_map.sensor_errors.baro_error = error ? 1 : 0;
    update_sensor_errors();
}

void RegisterMap::update_sensor_errors()
{
    // TODO: in case of error state flash LED.
}

void RegisterMap::set_baro_pressure(int32_t pressure)
{
    // register_map.baro_pressure[0] = static_cast<uint8_t>(pressure & 0xFF);
    // register_map.baro_pressure[1] = static_cast<uint8_t>((pressure >> 8) & 0xFF);
    // register_map.baro_pressure[2] = static_cast<uint8_t>((pressure >> 16) & 0xFF);

    // debug_print("new pressure data: %02X, %02X, %02X.\n",
        // register_map.baro_pressure[0],
        // register_map.baro_pressure[1],
        // register_map.baro_pressure[2]);

    // debug_print("led data: %02X.\n", register_map.red_led);
}

void RegisterMap::set_baro_temperature(int16_t temperature)
{
    // register_map.baro_temp = temperature;
}

} // namespace storage
