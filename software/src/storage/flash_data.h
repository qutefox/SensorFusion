#pragma once


#include <stdint.h>

#include "src/storage/flash_interface.h"

namespace storage
{

class FlashData
{
private:
    static FlashData* instance;
    static uint32_t lock;

    FlashInterface* flash;

protected:
    FlashData();
    virtual ~FlashData();

public:
    FlashData(FlashData& other) = delete;
    void operator=(const FlashData& other) = delete;

    static FlashData* get_instance();
};

} // namespace storage
