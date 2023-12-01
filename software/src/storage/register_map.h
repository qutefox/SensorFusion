#pragma once

#include "src/storage/register_map_interface.h"

namespace storage
{

class RegisterMap : public RegisterMapInterface
{
private:
    static RegisterMap* instance;
    static uint32_t lock;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* registers = nullptr;

    storage::RegisterInterface<uint8_t, uint8_t>* control_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* status_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* powermode_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* data_ready_register = nullptr;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* quaternion_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* euler_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* earth_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* gyroscope_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* accelerometer_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* magnetometer_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* pressure_data_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* temperature_data_register = nullptr;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* gyroscope_misalignment_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* gyroscope_sensitivity_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* gyroscope_offset_register = nullptr;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* accelerometer_misalignment_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* accelerometer_sensitivity_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* accelerometer_offset_register = nullptr;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* soft_iron_matrix_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* hard_iron_offset_register = nullptr;

protected:
    RegisterMap();
    virtual ~RegisterMap();

public:
    RegisterMap(RegisterMap& other) = delete;
    void operator=(const RegisterMap& other) = delete;

    static RegisterMap* get_instance();

public:
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_base() const override;
    
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_control_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_status_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_powermode_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_data_ready_register() const override;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_quaternion_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_euler_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_earth_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_magnetometer_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_pressure_data_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_temperature_data_register() const override;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_misalignment_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_sensitivity_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_offset_register() const override;

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_misalignment_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_sensitivity_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_offset_register() const override;
    
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_soft_iron_matrix_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_hard_iron_offset_register() const override;
};

} // namespace storage
