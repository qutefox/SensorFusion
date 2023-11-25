#include "src/storage/flash.h"

#include <cstring>
#include <cstdlib>

#include "mxc_errors.h"
#include "mxc_lock.h"
#include "nvic_table.h"
#include "flc.h"
#include "icc.h"

using namespace storage;

Flash* Flash::instance = nullptr;
uint32_t Flash::lock = 0;

#define LAST_FLASH_PAGE_ADDRESS (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE) - (1 * MXC_FLASH_PAGE_SIZE)
/*
^ Points to last page in flash, which is guaranteed to be unused.
For larger applications it's recommended to reserve a dedicated flash
region by creating a modified linkerfile.
*/

Flash::Flash()
{
    NVIC_SetRAM();
    // Not sure but: clear interrupt done and interrupt alternate function flags.
    MXC_FLC_ClearFlags(0x3);
}

Flash::~Flash()
{

}

Flash* Flash::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Flash();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Flash::read(uint8_t address, uint8_t* out_buffer, uint8_t length)
{
    MXC_FLC_Read(LAST_FLASH_PAGE_ADDRESS + address, out_buffer, length);
    return E_NO_ERROR;
}

int Flash::write(uint8_t address, const uint8_t* in_buffer, uint8_t length)
{
    int err = E_NO_ERROR;

    // Check if flash controller is busy
    if (MXC_FLC0->ctrl & MXC_F_FLC_CTRL_BUSY)
    {
        return E_BUSY;
    }

    // Disable the instruction cache controller.
    // Any code that modifies flash contents should disable the ICC,
    // since modifying flash contents may invalidate cached instructions.
    MXC_ICC_Disable();

    /*
    To modify a location in flash that has already been written to,
    that location must first be restored to its erased state.
    However, the flash controller only supports erasing a full page
    at a time.
    Therefore, the entire page must be buffered, erased, then modified.
    */

    // Buffering page.
    // 8192 bytes per page / 4 bytes = 2048 uint32_t
    uint32_t tmp_buffer[MXC_FLASH_PAGE_SIZE >> 2] = { 0xFFFFFFFF };
    MXC_FLC_Read(LAST_FLASH_PAGE_ADDRESS, tmp_buffer, MXC_FLASH_PAGE_SIZE);

    // Erasing page.
    err = MXC_FLC_PageErase(LAST_FLASH_PAGE_ADDRESS);
    if (err != E_NO_ERROR) return err;

    // Write data to buffer at specified adddress.
    memcpy(tmp_buffer+address, in_buffer, length);

    // Re-writing page from buffer.
    for (unsigned int i = 0; i < (MXC_FLASH_PAGE_SIZE >> 2); ++i)
    {
        err |= MXC_FLC_Write32(LAST_FLASH_PAGE_ADDRESS + 4 * i, tmp_buffer[i]);
    }

    // Verify.
    // read() will do the address shift and write to our buffer from the beginning!
    // So in the next loop our index is starting from 0.

    // We want to access buffers by bytes.
    uint8_t* byte_tmp_buffer = reinterpret_cast<uint8_t*>(tmp_buffer);
    const uint8_t* in_byte_buffer = reinterpret_cast<const uint8_t*>(in_buffer);

    read(address, byte_tmp_buffer, length);

    // Match read-back with what we should have written.
    for (int i = 0 ; i < length ; ++i)
    {
        if (byte_tmp_buffer[i] != in_byte_buffer[i])
        {
            err |= E_FAIL;
        }
    }

    // Enable the instruction cache controller.
    MXC_ICC_Enable();
    return err;
}

