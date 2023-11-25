#pragma once

#include "register.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class RegisterWithReadFlag : public Register<RegisterType, AddressType>
{
protected:
    bool read_flag = false;

public:
    RegisterWithReadFlag(RegisterType write_mask, RegisterType value)
        : Register<RegisterType, AddressType>(write_mask, value)
        , read_flag{ false }
    { }
    virtual ~RegisterWithReadFlag() { }

    virtual void set_read_flag() override
    {
        read_flag = true;
    }

    virtual bool get_read_flag() const override
    {
        return read_flag;
    }

    virtual void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = this->value;
        if (mark_read) read_flag = true;
    }

    virtual void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (use_write_mask) this->value = (this->value & (~this->write_mask)) | (write_value & this->write_mask);
        else this->value = write_value;
        read_flag = false;
    }
};

} // namespace storage
