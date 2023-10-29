#include "register_map.h"

#include <cstring>

#include "mxc_lock.h"

#include "src/debug_print.h"

namespace storage
{

// TODO: protection againt writing a register value while i2c slave reads it.

RegisterMap* RegisterMap::instance = nullptr;
uint32_t RegisterMap::lock = 0;

constexpr register_map_t write_enable_map = {
    { 0, 0, 0, 0, 0 }, // sensor_errors
    { 0, 0, 0, 0, 0 }, // data_ready
    { 1, 0 }, // red_led
    { 0, 0, 0}, // baro_pressure
    { 0 } // baro_temp
};

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
   memset(static_cast<void*>(&register_map), 0u, sizeof(register_map_t));
}

bool RegisterMap::is_address_in_range(uint8_t addr)
{
    return addr < sizeof(register_map);
}

uint8_t RegisterMap::read(uint8_t addr)
{
    debug_print("RegisterMap: Reading register address: %02X.\n", addr);
    if (!is_address_in_range(addr)) return 0x00;
    uint8_t* rm = reinterpret_cast<uint8_t*>(&register_map);
    return rm[addr];
}

void RegisterMap::write(uint8_t addr, uint8_t value)
{
    debug_print("RegisterMap: Writing register address: %02X with value: %02X.\n", addr, value);
    if (!is_address_in_range(addr)) return;
    const uint8_t* wem = reinterpret_cast<const uint8_t*>(&write_enable_map);
    uint8_t* rm = reinterpret_cast<uint8_t*>(&register_map);
    rm[addr] = value & wem[addr];
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
    register_map.baro_pressure[0] = static_cast<uint8_t>(pressure & 0xFF);
    register_map.baro_pressure[1] = static_cast<uint8_t>((pressure >> 8) & 0xFF);
    register_map.baro_pressure[2] = static_cast<uint8_t>((pressure >> 16) & 0xFF);
}

void RegisterMap::set_baro_temperature(int16_t temperature)
{
    register_map.baro_temp = temperature;
}

} // namespace storage
