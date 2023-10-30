#include "register_map_read_marker.h"

#include <cstring>

namespace storage
{

RegisterMapReadMarker::RegisterMapReadMarker()
    : map{ false }
{

}

RegisterMapReadMarker::~RegisterMapReadMarker()
{

}

void RegisterMapReadMarker::clear()
{
    for (uint8_t i = 0 ; i < sizeof(register_map_t) ; ++i)
    {
        map[i] = false;
    }
}

bool RegisterMapReadMarker::is_address_in_range(uint8_t addr)
{
    return addr < sizeof(register_map_t);
}

void RegisterMapReadMarker::mark_register_address(uint8_t addr)
{
    if (!is_address_in_range(addr)) return;
    map[addr] = true;
}

bool RegisterMapReadMarker::get_next_marked_register_address(uint8_t& addr)
{
    addr = 0;
    for (uint8_t i = 0 ; i < sizeof(register_map_t) ; ++i)
    {
        if (map[i] != false)
        {
            map[i] = false;
            addr = i;
            return true;
        }
    }
    return false;
}

} // namespace storage
