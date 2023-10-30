#include "register_map_reader_writer.h"

#include <cstring>

#include "mxc_lock.h"

#include "src/debug_print.h"

namespace storage
{

RegisterMapReaderWriter* RegisterMapReaderWriter::instance = nullptr;
uint32_t RegisterMapReaderWriter::lock = 0;

RegisterMapReaderWriter::RegisterMapReaderWriter()
    : map{ 0 }
    , write_rule_map{
        { 0x00 }, // sensor_errors
        { 0x00 }, // data_ready
        { 0x01 }, // red_led
        { 0x00 }, { 0x00 }, { 0x00 }, // baro_pressure
        { 0x00 }, { 0x00 } // baro_temp
    }
{

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

void RegisterMapReaderWriter::reset(bool set_content_changed_flag)
{
    read_marker.clear();
    write_marker.clear();
    memset(&map[0], 0, sizeof(register_map_t));
}

bool RegisterMapReaderWriter::is_address_in_range(uint8_t addr)
{
    return addr < sizeof(register_map_t);
}

uint8_t RegisterMapReaderWriter::read(uint8_t addr, bool mark_read)
{
    if (!is_address_in_range(addr)) return 0x00;
    uint8_t data = map[addr];
    if (mark_read) read_marker.mark_register_address(addr);
    return data;
}

uint8_t RegisterMapReaderWriter::write(uint8_t addr, uint8_t value,
    bool check_write_rules, bool mark_changed_bits)
{
    if (!is_address_in_range(addr)) return 0;

    uint8_t mask = 0xFF;
    uint8_t prev_value = 0;
    
    if (check_write_rules)
    {
        mask = write_rule_map[addr];
        if (mask == 0) return 1;
    }

    prev_value = map[addr];
    map[addr] = value & mask;

    if (mark_changed_bits)
    {
        uint8_t changed_bits = prev_value ^ (value & mask);
        write_marker.mark_register_address(addr, changed_bits);
    }

    return 1;
}

uint8_t RegisterMapReaderWriter::write(uint8_t addr, const uint8_t* value, uint8_t len,
    bool check_write_rules, bool mark_changed_bits)
{
    uint8_t mask = 0xFF;
    uint8_t prev_value = 0;
    uint8_t changed_bits = 0;

    for (uint8_t i = 0; i < len ; ++i)
    {
        if (!is_address_in_range(addr+i)) return i;

        if (check_write_rules)
        {
            mask = write_rule_map[addr+i];
            if (mask == 0) continue;
        }

        prev_value = map[addr+i];
        map[addr+i] = value[i] & mask;

        if (mark_changed_bits)
        {
            changed_bits = prev_value ^ (value[i] & mask);
            write_marker.mark_register_address(addr+i, changed_bits);
        }
    }

    return len;
}

bool RegisterMapReaderWriter::get_next_written_reg(uint8_t& reg_addr, uint8_t& reg_changed_bits, uint8_t& reg_new_value)
{
    reg_new_value = 0;
    if(write_marker.get_next_marked_register_address(reg_addr, reg_changed_bits))
    {
        reg_new_value = map[reg_addr];
        return true;
    }
    return false;
}

bool RegisterMapReaderWriter::get_next_read_reg(uint8_t& reg_addr)
{
    return read_marker.get_next_marked_register_address(reg_addr);
}

} // namespace storage
