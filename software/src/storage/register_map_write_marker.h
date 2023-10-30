#pragma once

#include <stdint.h>

#include "src/storage/register_map_type.h"

namespace storage
{

class RegisterMapWriteMarker
{
    uint8_t map[sizeof(register_map_t)];

    bool is_address_in_range(uint8_t addr);

public:
    RegisterMapWriteMarker();
    ~RegisterMapWriteMarker();

    void clear();
    void mark_register_address(uint8_t addr, uint8_t changed_bits);
    bool get_next_marked_register_address(uint8_t& addr, uint8_t& changed_bits);
};

} // namespace storage
