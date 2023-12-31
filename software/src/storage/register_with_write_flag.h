#pragma once

#include "src/storage/register.h"
#include "src/debug_print.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class RegisterWithWriteFlag : public Register<RegisterType, AddressType>
{
protected:
    volatile RegisterType written_bit_mask = 0;

public:
    RegisterWithWriteFlag(RegisterType write_mask, RegisterType value)
        : Register<RegisterType, AddressType>(write_mask, value)
        , written_bit_mask{ 0 }
    { }
    virtual ~RegisterWithWriteFlag() { }

    virtual RegisterType get_written_bit_mask() override
    {
        RegisterType tmp = written_bit_mask;
        written_bit_mask = 0;
        return tmp;
    }

    virtual void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!this->write_enabled) return;
        RegisterType prev_value = this->value;

        if (use_write_mask) this->value = (this->value & (~this->write_mask)) | (write_value & this->write_mask);
        else this->value = write_value;

        if (mark_changed_bits) written_bit_mask = (prev_value ^ this->value) | written_bit_mask;

        // debug_print("write(%02X, %d, %d)\n", write_value, (use_write_mask?1:0), (mark_changed_bits?1:0));
        // debug_print("value=%02X, written_bit_mask=%02X.\n", this->value, written_bit_mask);
    }

    virtual void set_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!this->write_enabled) return;
        RegisterType prev_value = this->value;

        if (use_write_mask) this->value |= (this->value & (~this->write_mask)) | (bits_value & this->write_mask);
        else this->value |= bits_value;

        if (mark_changed_bits) written_bit_mask = (prev_value ^ this->value) | written_bit_mask;
    }

    virtual void clear_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!this->write_enabled) return;
        RegisterType prev_value = this->value;

        if (use_write_mask) this->value &= (this->value & (~this->write_mask)) | ((~bits_value) & this->write_mask);
        else this->value &= (~bits_value);

        if (mark_changed_bits) written_bit_mask = (prev_value ^ this->value) | written_bit_mask;
    }
};

} // namespace storage
