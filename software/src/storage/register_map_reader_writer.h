#pragma once

#include <stdint.h>

#include "src/storage/register_map_type.h"
#include "src/storage/register_map_read_marker.h"
#include "src/storage/register_map_write_marker.h"

namespace storage
{

class RegisterMapReaderWriter
{
private:
    static RegisterMapReaderWriter* instance;
    static uint32_t lock;

    uint8_t map[sizeof(register_map_t)];
    const uint8_t write_rule_map[sizeof(register_map_t)];
    RegisterMapReadMarker read_marker;
    RegisterMapWriteMarker write_marker;

protected:
    RegisterMapReaderWriter();
    ~RegisterMapReaderWriter();

public:
    RegisterMapReaderWriter(RegisterMapReaderWriter& other) = delete;
    void operator=(const RegisterMapReaderWriter& other) = delete;

    static RegisterMapReaderWriter* get_instance();

    void reset(bool set_content_changed_flag = true);

    bool is_address_in_range(uint8_t addr);

    uint8_t read(uint8_t addr, bool mark_read = true);

    uint8_t write(uint8_t addr, uint8_t value,
        bool check_write_rules = true, bool mark_changed_bits = true);

    uint8_t write(uint8_t addr, const uint8_t* value, uint8_t len,
        bool check_write_rules = true, bool mark_changed_bits = true);

    bool get_next_written_reg(uint8_t& reg_addr, uint8_t& reg_changed_bits, uint8_t& reg_new_value);

    bool get_next_read_reg(uint8_t& reg_addr);
};

} // namespace storage
