#pragma once

#include <type_traits>

namespace storage
{

template<typename RegisterType, typename AddressType>
class IRegister
{
    static_assert(std::is_integral<RegisterType>::value, "Wrong argument type. Only numeric types are accepted for register type.");
    static_assert(!std::is_same<RegisterType, bool>::value, "Wrong argument type. Bool is not accepted for register type.");

    static_assert(std::is_integral<AddressType>::value, "Wrong argument type. Only numeric types are accepted for address tpye.");
    static_assert(std::is_unsigned<AddressType>::value, "Wrong argument type. Only unsigned types are accepted for address tpye.");

public:
    IRegister() { }
    virtual ~IRegister() { }

    virtual inline bool get_read_flag() const = 0;
    virtual inline RegisterType get_written_bit_mask() = 0;

    virtual inline void read(RegisterType& value, bool mark_read=true) = 0;
    virtual inline bool read(RegisterType* buffer, AddressType length, bool mark_read=true) = 0;

    virtual inline void write(RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual inline bool write(RegisterType* buffer, AddressType length, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
};

template<typename RegisterType, typename AddressType>
class IMultiRegister : public IRegister<RegisterType, AddressType>
{
public:
    IMultiRegister() { }
    virtual ~IMultiRegister() { }

    virtual inline void read(RegisterType* buffer, bool mark_read=true) = 0;
    virtual inline bool read(AddressType offset, RegisterType& value, bool mark_read=true) = 0;
    virtual inline bool read(AddressType offset, RegisterType* buffer, AddressType length, bool mark_read=true) = 0;

    virtual inline void write(RegisterType* buffer, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual inline bool write(AddressType offset, RegisterType value, bool use_write_mask=true, bool mark_changed_bits=true) = 0;
    virtual inline bool write(AddressType offset, RegisterType* buffer, AddressType length, bool use_write_mask=true, bool mark_changed_bits=true) = 0;

    virtual inline IRegister<RegisterType, AddressType>* get_register(AddressType offset) const = 0;

    virtual inline bool get_read_flag(bool& read_flag, AddressType offset) = 0;
    virtual inline bool get_written_bit_mask(RegisterType& written_bit_mask, AddressType offset) = 0;
    virtual inline bool is_read() const = 0;

    virtual inline bool get_next_written_and_changed_register(AddressType& offset, RegisterType& written_bit_mask, RegisterType& new_value) = 0;
};

} // namespace storage
