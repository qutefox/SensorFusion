#pragma once

#include "register_interface.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class Register : public RegisterInterface<RegisterType, AddressType>
{
protected:
    volatile RegisterType write_mask = 0;
    volatile RegisterType value = 0;
    volatile bool write_enabled = true; /** Disables register writes while keeping the write mask unchanged. */

public:
    Register(RegisterType write_mask, RegisterType value)
        : write_mask{ write_mask }
        , value{ value }
        , write_enabled{ true }
    { }
    virtual ~Register() { }

    virtual void set_read_flag() override
    {

    }

    virtual bool get_read_flag() const override
    {
        return false;
    }

    virtual RegisterType get_written_bit_mask() override
    {
        return 0;
    }

    virtual bool is_write_enabled() override
    {
        return write_enabled;
    }

    virtual void set_write_enabled(bool value) override
    {
        write_enabled = value;
    }

    virtual void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = value;
    }

    virtual bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length != 1) return false;
        read(buffer[0], mark_read);
        return true;
    }

    virtual void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!write_enabled) return;
        if (use_write_mask) value = (value & (~write_mask)) | (write_value & write_mask);
        else value = write_value;
    }

    virtual bool write(const RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!write_enabled || write_length != 1) return false;
        write(buffer[0], use_write_mask, mark_changed_bits);
        return true;
    }

    virtual void set_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!write_enabled) return;
        if (use_write_mask) value |= (value & (~write_mask)) | (bits_value & write_mask);
        else value |= bits_value;
    }

    virtual void clear_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (!write_enabled) return;
        if (use_write_mask) value &= (value & (~write_mask)) | ((~bits_value) & write_mask);
        else value &= (~bits_value);
    }
};

} // namespace storage
