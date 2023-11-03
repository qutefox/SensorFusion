#pragma once

#include <cstdlib>
#include <functional>

#include "register_interface.h"

namespace storage
{

template<typename RegisterType, typename AddressType>
class Register : public RegisterInterface<RegisterType, AddressType>
{
protected:
    RegisterType write_mask = 0;
    RegisterType value = 0;

public:
    Register(RegisterType write_mask, RegisterType value)
        : write_mask{ write_mask }
        , value{ value }
    { }
    virtual ~Register() { }

    virtual inline bool get_read_flag() const override
    {
        return false;
    }

    virtual inline RegisterType get_written_bit_mask() override
    {
        return 0;
    }

    virtual inline void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = value;
    }

    virtual inline bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length != 1) return false;
        read(buffer[0], mark_read);
        return true;
    }

    virtual inline void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (use_write_mask) value = (value & (~write_mask)) | (write_value & write_mask);
        else value = write_value;
    }

    virtual inline bool write(RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length != 1) return false;
        write(buffer[0], use_write_mask, mark_changed_bits);
        return true;
    }
};

template<typename RegisterType, typename AddressType>
class RegisterWithReadFlag : public RegisterInterface<RegisterType, AddressType>
{
protected:
    RegisterType write_mask = 0;
    RegisterType value = 0;
    bool read_flag = false;

public:
    RegisterWithReadFlag(RegisterType write_mask, RegisterType value)
        : write_mask{ write_mask }
        , value{ value }
        , read_flag{ false }
    { }
    virtual ~RegisterWithReadFlag() { }

    virtual inline bool get_read_flag() const override
    {
        return read_flag;
    }

    virtual inline RegisterType get_written_bit_mask() override
    {
        return 0;
    }

    virtual inline void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = value;
        if (mark_read) read_flag = true;
    }

    virtual inline bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length != 1) return false;
        read(buffer[0], mark_read);
        return true;
    }

    virtual inline void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (use_write_mask) value = (value & (~write_mask)) | (write_value & write_mask);
        else value = write_value;
        read_flag = false;
    }

    virtual inline bool write(RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length != 1) return false;
        write(buffer[0], use_write_mask, mark_changed_bits);
        return true;
    }
};

template<typename RegisterType, typename AddressType>
class RegisterWithReadWriteFlag : public RegisterInterface<RegisterType, AddressType>
{
protected:
    RegisterType write_mask = 0;
    RegisterType value = 0;
    bool read_flag = false;
    RegisterType written_bit_mask = 0;

public:
    RegisterWithReadWriteFlag(RegisterType write_mask, RegisterType value)
        : write_mask{ write_mask }
        , value{ value }
        , read_flag{ false }
        , written_bit_mask{ 0 }
    { }
    virtual ~RegisterWithReadWriteFlag() { }

    virtual inline bool get_read_flag() const override
    {
        return read_flag;
    }

    virtual inline RegisterType get_written_bit_mask() override
    {
        RegisterType tmp = written_bit_mask;
        written_bit_mask = 0;
        return tmp;
    }

    virtual inline void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = value;
        if (mark_read) read_flag = true;
    }

    virtual inline bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length != 1) return false;
        read(buffer[0], mark_read);
        return true;
    }

    virtual inline void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        RegisterType prev_value = value;

        if (use_write_mask) value = (value & (~write_mask)) | (write_value & write_mask);
        else value = write_value;

        if (mark_changed_bits) written_bit_mask = (prev_value ^ value) | written_bit_mask;
        read_flag = false;
    }

    virtual inline bool write(RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length != 1) return false;
        write(buffer[0], use_write_mask, mark_changed_bits);
        return true;
    }
};

template<typename RegisterType, typename AddressType>
class RegisterWithWriteFlag : public RegisterInterface<RegisterType, AddressType>
{
protected:
    RegisterType write_mask = 0;
    RegisterType value = 0;
    RegisterType written_bit_mask = 0;

public:
    RegisterWithWriteFlag(RegisterType write_mask, RegisterType value)
        : write_mask{ write_mask }
        , value{ value }
        , written_bit_mask{ 0 }
    { }
    virtual ~RegisterWithWriteFlag() { }

    virtual inline bool get_read_flag() const override
    {
        return false;
    }

    virtual inline RegisterType get_written_bit_mask() override
    {
        RegisterType tmp = written_bit_mask;
        written_bit_mask = 0;
        return tmp;
    }

    virtual inline void read(RegisterType& read_value, bool mark_read=true) override
    {
        read_value = value;
    }

    virtual inline bool read(RegisterType* buffer, AddressType read_length, bool mark_read=true) override
    {
        if (read_length != 1) return false;
        read(buffer[0], mark_read);
        return true;
    }

    virtual inline void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        RegisterType prev_value = value;

        if (use_write_mask) value = (value & (~write_mask)) | (write_value & write_mask);
        else value = write_value;

        if (mark_changed_bits) written_bit_mask = (prev_value ^ value) | written_bit_mask;
    }

    virtual inline bool write(RegisterType* buffer, AddressType write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length != 1) return false;
        write(buffer[0], use_write_mask, mark_changed_bits);
        return true;
    }
};

template<typename RegisterType, typename N>
class MultiRegister : public MultiRegisterInterface<RegisterType, N>
{
protected:
    N length = 0;
    bool has_parent = false;
    RegisterInterface<RegisterType, N>** registers = nullptr;

public:
    MultiRegister(N length, bool has_parent, std::function<void(RegisterInterface<RegisterType, N>** registers, N length)> initaliser_func)
        : length{ length }
        , has_parent{ has_parent }
        , registers{ nullptr }
    {
        registers = static_cast<RegisterInterface<RegisterType, N>**>(malloc(sizeof(RegisterInterface<RegisterType, N>*) * length));
        for (N i = 0; i < length; ++i)
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

        for (N i = 0; i < length; ++i)
        {
            if (registers[i]) delete registers[i];
        }
        free(registers);
    }

    virtual inline bool get_read_flag() const override
    {
        return registers[0]->get_read_flag();
    }

    virtual inline bool get_read_flag(bool& _read_flag, N offset) override
    {
        if (offset >= length) return false;
        _read_flag = registers[offset]->get_read_flag();
        return true;
    }

    virtual inline bool is_read() const override
    {
        bool read_flag = true;
        for (N i = 0; i < length; ++i)
        {
            read_flag &= registers[i]->get_read_flag();
        }
        return read_flag;
    }

    virtual inline RegisterType get_written_bit_mask() override
    {
        return registers[0]->get_written_bit_mask();
    }

    virtual inline bool get_written_bit_mask(RegisterType& written_bit_mask, N offset) override
    {
        if (offset >= length) return false;
        written_bit_mask = registers[offset]->get_written_bit_mask();
        return true;
    }

    virtual inline bool get_next_written_and_changed_register(N& offset, RegisterType& written_bit_mask, RegisterType& new_value) override
    {
        for (N i = 0; i < length; ++i)
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

    virtual inline RegisterInterface<RegisterType, N>* get_register(N offset) const override
    {
        if (offset >= length) return nullptr;
        return registers[offset];
    }

    virtual inline bool read(N offset, RegisterType& read_value, bool mark_read=true) override
    {
        if (offset >= length)
        {
            read_value = 0;
            return false;
        }
        registers[offset]->read(read_value, mark_read);
        return true;
    }

    virtual inline bool read(N offset, RegisterType* buffer, N read_length, bool mark_read=true) override
    {
        if ((offset + read_length) > length) return false;
        for (N i = 0; i < read_length; ++i)
        {
            registers[offset + i]->read(buffer[i], mark_read);
        }
        return true;
    }

    virtual inline bool write(N offset, RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (offset >= length) return false;
        registers[offset]->write(value, use_write_mask, mark_changed_bits);
        return true;
    }

    virtual inline bool write(N offset, RegisterType* buffer, N write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if ((offset + write_length) > length) return false;
        for (N i = 0; i < write_length; ++i)
        {
            registers[offset+i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
        return true;
    }

    virtual inline void read(RegisterType& read_value, bool mark_read=true) override
    {
        registers[0]->read(read_value, mark_read);
    }

    virtual inline void read(RegisterType* buffer, bool mark_read=true) override
    {
        for (N i = 0; i < length; ++i)
        {
            registers[i]->read(buffer[i], mark_read);
        }
    }

    virtual inline bool read(RegisterType* buffer, N read_length, bool mark_read=true) override
    {
        if (read_length >= length) return false;
        for (N i = 0; i < read_length; ++i)
        {
            registers[i]->read(buffer[i], mark_read);
        }
        return true;
    }

    virtual inline void write(RegisterType write_value, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        registers[0]->write(write_value, use_write_mask, mark_changed_bits);
    }

    virtual inline void write(RegisterType* buffer, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        for (N i = 0; i < length; ++i)
        {
            registers[i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
    }

    virtual inline bool write(RegisterType* buffer, N write_length, bool use_write_mask=true, bool mark_changed_bits=true) override
    {
        if (write_length >= length) return false;
        for (N i = 0; i < write_length; ++i)
        {
            registers[i]->write(buffer[i], use_write_mask, mark_changed_bits);
        }
        return true;
    }
};

} // namespace storage
