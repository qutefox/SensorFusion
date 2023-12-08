#pragma once

#include <stdint.h>

#include "src/storage/register_interface.h"

namespace storage
{

class RegisterMapInterface
{
public:
    RegisterMapInterface() { }
    virtual ~RegisterMapInterface() { }

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_base() const = 0;

    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_board_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_control_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_status_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_powermode_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_data_ready_register() const = 0;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_quaternion_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_euler_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_earth_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_magnetometer_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_pressure_data_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_temperature_data_register() const = 0;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_misalignment_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_sensitivity_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_offset_register() const = 0;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_misalignment_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_sensitivity_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_offset_register() const = 0;
    
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_soft_iron_matrix_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_hard_iron_offset_register() const = 0;
};

} // namespace storage
