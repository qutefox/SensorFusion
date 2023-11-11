#pragma once

#include <stdint.h>

#include "src/storage/register_interface.h"

class RegistermapInterface
{
public:
    RegistermapInterface() { }
    virtual ~RegistermapInterface() { }

    virtual void reset() = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_addressable_base() const = 0;

    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_board_control_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_fusion_control_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_fusion_status_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_control_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_status_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_calibration_control_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_sensor_calibration_status_register() const = 0;
    virtual storage::RegisterInterface<uint8_t, uint8_t>* get_data_ready_register() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_gyroscope_fusion_registers() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_accelerometer_fusion_registers() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_magnetometer_fusion_registers() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_quaternion_fusion_registers() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_pressure_registers() const = 0;
    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_temperature_registers() const = 0;
};
