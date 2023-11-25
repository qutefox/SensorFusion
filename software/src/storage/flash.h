#pragma once

#include <stdint.h>

#include "src/storage/flash_interface.h"

namespace storage
{

class Flash : public FlashInterface
{
private:
    static Flash* instance;
    static uint32_t lock;
    
protected:
    Flash();
    virtual ~Flash();

public:
    Flash(Flash& other) = delete;
    void operator=(const Flash& other) = delete;

    static Flash* get_instance();

    virtual int read(uint8_t address, uint8_t* out_buffer, uint8_t length) override;
    virtual int write(uint8_t address, const uint8_t* in_buffer, uint8_t length) override;
    
};

} // namespace storage
