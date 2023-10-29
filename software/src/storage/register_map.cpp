#include "register_map.h"

#include <cstring>

#include "mxc_lock.h"

namespace storage
{

// TODO: protection againt writing a register value while i2c slave reads it.

RegisterMap* RegisterMap::instance = nullptr;
uint32_t RegisterMap::lock = 0;

RegisterMap::RegisterMap()
{
    reset();
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

void RegisterMap::reset()
{
    /*
    uint8_t* regs = reinterpret_cast<uint8_t*>(&register_map);
    regs[0] = 0x00; // sensor_errors
    regs[1] = 0x00; // data_ready
    regs[2] = 0x00; // red_led
    regs[3] = 0x00; // baro_temp[0]
    regs[4] = 0x00; // baro_temp[1]
    */

   // At this point we have more zeroes than ones.
   // Easier strategy:
   // memset all to zero and later flip stuff we need to be one.
   memset(static_cast<void*>(&register_map), 0u, sizeof(register_map_t));
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
    register_map.baro_temp = temperature;
}

} // namespace storage
