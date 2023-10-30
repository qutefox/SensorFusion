#pragma once

#include <stdint.h>

#include "src/storage/register_map_type.h"

namespace storage
{

class RegisterMapReadMarker
{
    bool map[sizeof(register_map_t)];

    bool is_address_in_range(uint8_t addr);

public:
    RegisterMapReadMarker();
    ~RegisterMapReadMarker();

    void clear();
    void mark_register_address(uint8_t addr);
    bool get_next_marked_register_address(uint8_t& addr);

};

} // namespace storage
