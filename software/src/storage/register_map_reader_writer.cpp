#include "register_map_reader_writer.h"

#include <cstring>

#include "mxc_lock.h"

#include "src/debug_print.h"

namespace storage
{

RegisterMapReaderWriter* RegisterMapReaderWriter::instance = nullptr;
uint32_t RegisterMapReaderWriter::lock = 0;

constexpr register_map_t write_enable_map = {
    { 0, 0, 0, 0, 0 }, // sensor_errors
    { 0, 0, 0, 0, 0 }, // data_ready
    { 1, 0 }, // red_led
    { 0, 0, 0}, // baro_pressure
    { 0 } // baro_temp
};

RegisterMapReaderWriter::RegisterMapReaderWriter()
    : content_changed{ false }
{
    reset();
}

RegisterMapReaderWriter::~RegisterMapReaderWriter()
{

}

RegisterMapReaderWriter* RegisterMapReaderWriter::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new RegisterMapReaderWriter();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int RegisterMapReaderWriter::begin()
{
    return E_NO_ERROR;
}

void RegisterMapReaderWriter::reset()
{
    // TODO: this might cause issues
    // if we reset we might want to get a change notification that stuff has been overwritten.
    memset(static_cast<void*>(&change_map), 0u, sizeof(register_map_t));
    memset(static_cast<void*>(&register_map), 0u, sizeof(register_map_t));
}

void RegisterMapReaderWriter::write_changed_content(uint8_t addr, uint8_t changed_bits)
{
    if (changed_bits == 0) return;
    content_changed = true;
    uint8_t* map = reinterpret_cast<uint8_t*>(&change_map);
    map[addr] = changed_bits;
}

bool RegisterMapReaderWriter::get_next_write_change(uint8_t& addr, uint8_t& changed_bits, uint8_t& new_value)
{
    addr = 0;
    changed_bits = 0;
    new_value = 0;

    uint8_t* c_map = reinterpret_cast<uint8_t*>(&change_map);
    uint8_t* r_map = reinterpret_cast<uint8_t*>(&register_map);

    for (uint8_t i = 0 ; i < sizeof(register_map_t) ; ++i)
    {
        changed_bits = c_map[i];
        if (changed_bits == 0) continue;
        c_map[i] = 0;
        new_value = r_map[i];
        addr = i;
        return true;
    }
    return false;
}

bool RegisterMapReaderWriter::is_address_in_range(uint8_t addr)
{
    return addr < sizeof(register_map_t);
}

uint8_t RegisterMapReaderWriter::read(uint8_t addr)
{
    if (!is_address_in_range(addr)) return 0x00;
    uint8_t* map = reinterpret_cast<uint8_t*>(&register_map);
    uint8_t data = map[addr];
    return data;
}

uint8_t RegisterMapReaderWriter::write(uint8_t addr, uint8_t value, bool check_write_rules)
{
    if (!is_address_in_range(addr)) return 0;

    const uint8_t* wem = reinterpret_cast<const uint8_t*>(&write_enable_map);
    uint8_t* map = reinterpret_cast<uint8_t*>(&register_map);
    uint8_t mask = 0xFF;
    uint8_t prev_value = 0;
    
    if (check_write_rules)
    {
        mask = wem[addr];
        if (mask == 0) return 1;
        prev_value = map[addr];
    }

    map[addr] = value & mask;

    if (check_write_rules)
    {
        uint8_t changed_bits = prev_value ^ (value & mask);
        write_changed_content(addr, changed_bits);
    }

    return 1;
}

uint8_t RegisterMapReaderWriter::write(uint8_t addr, const uint8_t* value, uint8_t len, bool check_write_rules)
{
    const uint8_t* wem = reinterpret_cast<const uint8_t*>(&write_enable_map);
    uint8_t* map = reinterpret_cast<uint8_t*>(&register_map);
    uint8_t mask = 0xFF;
    uint8_t prev_value = 0;
    uint8_t changed_bits = 0;

    for (uint8_t i = 0; i < len ; ++i)
    {
        if (!is_address_in_range(addr+i)) return i;

        if (check_write_rules)
        {
            mask = wem[addr+i];
            if (mask == 0) continue;
            prev_value = map[addr+i];
        }

        map[addr+i] = value[i] & mask;

        if (check_write_rules)
        {
            changed_bits = prev_value ^ (value[i] & mask);
            write_changed_content(addr+i, changed_bits);
        }
    }

    return len;
}

} // namespace storage
