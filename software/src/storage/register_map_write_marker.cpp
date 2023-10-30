#include "register_map_write_marker.h"

#include <cstring>

namespace storage
{

RegisterMapWriteMarker::RegisterMapWriteMarker()
    : map{ 0 }
{

}

RegisterMapWriteMarker::~RegisterMapWriteMarker()
{

}

void RegisterMapWriteMarker::clear()
{
    memset(&map[0], 0, sizeof(register_map_t));
}

bool RegisterMapWriteMarker::is_address_in_range(uint8_t addr)
{
    return addr < sizeof(register_map_t);
}

void RegisterMapWriteMarker::mark_register_address(uint8_t addr, uint8_t changed_bits)
{
    if (changed_bits == 0 || !is_address_in_range(addr)) return;
    map[addr] = changed_bits;
}

bool RegisterMapWriteMarker::get_next_marked_register_address(uint8_t& addr, uint8_t& changed_bits)
{
    addr = 0;
    changed_bits = 0;
    for (uint8_t i = 0 ; i < sizeof(register_map_t) ; ++i)
    {
        if (map[i] != 0)
        {
            map[i] = 0;
            addr = i;
            return true;
        }
    }
    return false;
}

} // namespace storage
