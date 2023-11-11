#pragma once

#include "src/register_map_interface.h"

class Registermap : public RegistermapInterface
{
private:
    static Registermap* instance;
    static uint32_t lock;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* registers = nullptr;

    storage::RegisterInterface<uint8_t, uint8_t>* board_control_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* fusion_control_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* fusion_status_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* sensor_control_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* sensor_status_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* sensor_calibration_control_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* sensor_calibration_status_register = nullptr;
    storage::RegisterInterface<uint8_t, uint8_t>* data_ready_register = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* gyroscope_fusion_registers = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* accelerometer_fusion_registers = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* magnetometer_fusion_registers = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* quaternion_fusion_registers = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* pressure_registers = nullptr;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* temperature_registers = nullptr;

protected:
    Registermap();
    virtual ~Registermap();

public:
    Registermap(Registermap& other) = delete;
    void operator=(const Registermap& other) = delete;

    static Registermap* get_instance();

public:
    virtual void reset() override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_addressable_base() const override;
    
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_board_control_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_fusion_control_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_fusion_status_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_control_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_status_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_calibration_control_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_calibration_status_register() const override;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_data_ready_register() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_fusion_registers() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_fusion_registers() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_magnetometer_fusion_registers() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_quaternion_fusion_registers() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_pressure_registers() const override;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_temperature_registers() const override;
};