#pragma once

#include <stdint.h>

#include "src/storage/register_map_type.h"

namespace storage
{

class RegisterMapReaderWriter
{
private:
    static RegisterMapReaderWriter* instance;
    static uint32_t lock;

    register_map_t register_map;
    register_map_t change_map;
    bool content_changed = false;

    void write_changed_content(uint8_t addr, uint8_t changed_bits);

protected:
    RegisterMapReaderWriter();
    ~RegisterMapReaderWriter();

public:
    RegisterMapReaderWriter(RegisterMapReaderWriter& other) = delete;
    void operator=(const RegisterMapReaderWriter& other) = delete;

    static RegisterMapReaderWriter* get_instance();

    int begin();

    void reset();

    bool is_address_in_range(uint8_t addr);
    uint8_t read(uint8_t addr);
    uint8_t write(uint8_t addr, uint8_t value, bool check_write_rules = true);
    uint8_t write(uint8_t addr, const uint8_t* value, uint8_t len, bool check_write_rules = true);

    bool get_next_write_change(uint8_t& addr, uint8_t& changed_bits, uint8_t& new_value);
};

} // namespace storage
