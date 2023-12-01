#pragma once

#include "register.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class RegisterWithReadWriteFlag : public Register<RegisterType, AddressType>
{
protected:
    volatile bool read_flag = false;
    volatile RegisterType written_bit_mask = 0;

public:
    RegisterWithReadWriteFlag(RegisterType write_mask, RegisterType value)
        : Register<RegisterType, AddressType>(write_mask, value)
        , read_flag{ false }
        , written_bit_mask{ 0 }
    { }
    virtual ~RegisterWithReadWriteFlag() { }

    virtual void set_read_flag() override
    {
        read_flag = true;
    }

    virtual bool get_read_flag() const override
    {
        return read_flag;
    }

    virtual RegisterType get_written_bit_mask() override
    {
        RegisterType tmp = written_bit_mask;
        written_bit_mask = 0;
        return tmp;
    }

    virtual void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = this->value;
        if (mark_read) read_flag = true;
    }

    virtual void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!this->write_enabled) return;
        RegisterType prev_value = this->value;

        if (use_write_mask) this->value = (this->value & (~this->write_mask)) | (write_value & this->write_mask);
        else this->value = write_value;

        if (mark_changed_bits) written_bit_mask = (prev_value ^ value) | written_bit_mask;
        read_flag = false;
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
