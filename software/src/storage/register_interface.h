#pragma once

#include <type_traits>

namespace storage
{

template<typename RegisterType, typename AddressType>
class RegisterInterface
{
    static_assert(std::is_integral<RegisterType>::value, "Wrong argument type. Only numeric types are accepted for register type.");
    static_assert(!std::is_same<RegisterType, bool>::value, "Wrong argument type. Bool is not accepted for register type.");

    static_assert(std::is_integral<AddressType>::value, "Wrong argument type. Only numeric types are accepted for address tpye.");
    static_assert(std::is_unsigned<AddressType>::value, "Wrong argument type. Only unsigned types are accepted for address tpye.");

public:
    RegisterInterface() { }
    virtual ~RegisterInterface() { }

    virtual void set_read_flag() = 0;
    virtual bool get_read_flag() const = 0;
    virtual RegisterType get_written_bit_mask() = 0;
    virtual bool is_write_enabled() = 0;
    virtual void set_write_enabled(bool value) = 0;

    virtual void read(RegisterType& value, bool mark_read=true) = 0;
    virtual bool read(RegisterType* buffer, AddressType length, bool mark_read=true) = 0;

    virtual void write(RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual bool write(const RegisterType* buffer, AddressType length, bool use_write_mask=true, bool mark_changed_bits=true) = 0;

    virtual void set_bits(RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual void clear_bits(RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
};

template<typename RegisterType, typename AddressType>
class MultiRegisterInterface : public RegisterInterface<RegisterType, AddressType>
{
public:
    MultiRegisterInterface() { }
    virtual ~MultiRegisterInterface() { }

    virtual AddressType size() const = 0;
    virtual bool is_valid_address(AddressType address) const = 0;

    virtual void read(RegisterType* buffer, bool mark_read=true) = 0;
    virtual bool read(AddressType offset, RegisterType& value, bool mark_read=true) = 0;
    virtual bool read(AddressType offset, RegisterType* buffer, AddressType length, bool mark_read=true) = 0;

    virtual void write(const RegisterType* buffer, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual bool write(AddressType offset, RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual bool write(AddressType offset, RegisterType* buffer, AddressType length, bool use_write_mask=true, bool mark_changed_bits=true) = 0;

    virtual RegisterInterface<RegisterType, AddressType>* get_register(AddressType offset) const = 0;

    virtual bool get_read_flag(bool& read_flag, AddressType offset) = 0;
    virtual bool get_written_bit_mask(RegisterType& written_bit_mask, AddressType offset) = 0;
    virtual bool is_read() const = 0;
    virtual void set_read() = 0;

    virtual bool get_next_written_and_changed_register(AddressType& offset, RegisterType& written_bit_mask, RegisterType& new_value) = 0;
};

} // namespace storage
