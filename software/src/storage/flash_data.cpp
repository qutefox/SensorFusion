#include "src/storage/flash_data.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/flash.h"
#include "src/debug_print.h"

using namespace storage;

FlashData* FlashData::instance = nullptr;
uint32_t FlashData::lock = 0;

FlashData::FlashData()
    : flash{ Flash::get_instance() }
{   

}

FlashData::~FlashData()
{

}

FlashData* FlashData::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new FlashData();
    }
    MXC_FreeLock(&lock);
    return instance;
}

// void FlashData::test()
// {
//     uint8_t td2[100] = {0};
//     flash->read(0, td2, 13);
//     td2[12] = '\0';
//     debug_print("Test result1: %s.\n", td2);
// 
//     uint8_t td[100] = "Hello World!\0";
//     int err = flash->write(0, td, 13);
// 
//     flash->read(0, td2, 13);
// 
//     debug_print("Test result2: %s. Write return code: %d.\n", td2, err);
// }