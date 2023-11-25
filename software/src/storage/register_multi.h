#pragma once

#include <cstdlib>
#include <functional>

#include "register_interface.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class MultiRegister : public MultiRegisterInterface<RegisterType, AddressType>
{
protected:
    AddressType length = 0;
    bool has_parent = false;
    RegisterInterface<RegisterType, AddressType>** registers = nullptr;

public:
    MultiRegister(AddressType length, bool has_parent, std::function<void(RegisterInterface<RegisterType, AddressType>** registers, AddressType length)> initaliser_func)
        : length{ length }
        , has_parent{ has_parent }
        , registers{ nullptr }
    {
        registers = static_cast<RegisterInterface<RegisterType, AddressType>**>(malloc(sizeof(RegisterInterface<RegisterType, AddressType>*) * length));
        for (AddressType i = 0; i < length; ++i)
        {
            registers[i] = nullptr;
        }
        initaliser_func(registers, length);
    }

    virtual ~MultiRegister()
    {
        if (has_parent)
        {
            // If config is good then parent will take care of the register elements.
            free(registers);
            return;
        }

        for (AddressType i = 0; i < length; ++i)
        {
            if (registers[i]) delete registers[i];
        }
        free(registers);
    }

    virtual AddressType size() const override
    {
        return length;
    }

    virtual bool is_valid_address(AddressType address) const
    {
        return address < length;
    }

    virtual void set_read_flag() override
    {
        registers[0]->set_read_flag();
    }

    virtual bool get_read_flag() const override
    {
        return registers[0]->get_read_flag();
    }

    virtual bool get_read_flag(bool& _read_flag, AddressType offset) override
    {
        if (offset >= length) return false;
        _read_flag = registers[offset]->get_read_flag();
        return true;
    }

    virtual bool is_read() const override
    {
        bool read_flag = true;
        for (AddressType i = 0; i < length; ++i)
        {
            read_flag &= registers[i]->get_read_flag();
        }
        return read_flag;
    }

    virtual void set_read() override
    {
        for (AddressType i = 0; i < length; ++i)
        {
            registers[i]->set_read_flag();
        }
    }

    virtual RegisterType get_written_bit_mask() override
    {
        return registers[0]->get_written_bit_mask();
    }

    virtual bool get_written_bit_mask(RegisterType& written_bit_mask, AddressType offset) override
    {
        if (offset >= length) return false;
        written_bit_mask = registers[offset]->get_written_bit_mask();
        return true;
    }

    virtual bool get_next_written_and_changed_register(AddressType& offset, RegisterType& written_bit_mask, RegisterType& new_value) override
    {
        for (AddressType i = 0; i < length; ++i)
        {
            written_bit_mask = registers[i]->get_written_bit_mask();
            if (written_bit_mask)
            {
                registers[i]->read(new_value, false);
                offset = i;
                return true;
            }
        }
        return false;
    }

    virtual RegisterInterface<RegisterType, AddressType>* get_register(AddressType offset) const override
    {
        if (offset >= length) return nullptr;
        return registers[offset];
    }

    virtual bool read(AddressType offset, RegisterType& read_value, bool mark_read=true) override
    {
        if (offset >= length)
        {
            read_value = 0;
            return false;
        }
        registers[offset]->read(read_value, mark_read);
        return true;
    }

    virtual bool read(AddressType offset, RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if ((offset + read_length) > length) return false;
        for (AddressType i = 0; i < read_length; ++i)
        {
            registers[offset + i]->read(buffer[i], mark_read);
        }
        return true;
    }

    virtual bool write(AddressType offset, RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (offset >= length) return false;
        registers[offset]->write(value, use_write_mask, mark_changed_bits);
        return true;
    }

    virtual bool write(AddressType offset, RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if ((offset + write_length) > length) return false;
        for (AddressType i = 0; i < write_length; ++i)
        {
            registers[offset+i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
        return true;
    }

    virtual void read(RegisterType& read_value, bool mark_read=true) override
    {
        registers[0]->read(read_value, mark_read);
    }

    virtual void read(RegisterType* buffer, bool mark_read=true) override
    {
        for (AddressType i = 0; i < length; ++i)
        {
            registers[i]->read(buffer[i], mark_read);
        }
    }

    virtual bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length >= length) return false;
        for (AddressType i = 0; i < read_length; ++i)
        {
            registers[i]->read(buffer[i], mark_read);
        }
        return true;
    }

    virtual void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        registers[0]->write(write_value, use_write_mask, mark_changed_bits);
    }

    virtual void write(const RegisterType* buffer, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        for (AddressType i = 0; i < length; ++i)
        {
            registers[i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
    }

    virtual bool write(const RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length >= length) return false;
        for (AddressType i = 0; i < write_length; ++i)
        {
            registers[i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
        return true;
    }

    virtual void set_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        registers[0]->set_bits(bits_value, use_write_mask, mark_changed_bits);
    }

    virtual void clear_bits(RegisterType bits_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        registers[0]->clear_bits(bits_value, use_write_mask, mark_changed_bits);
    }
};

} // namespace storage
