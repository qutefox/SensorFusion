#pragma once

#include <stdint.h>

#include "src/storage/register_map_interface.h"
#include "src/sensor/sensor_types.h"
#include "src/storage/register_fields.h"
#include "src/Fusion/Fusion/Fusion.h"

namespace storage
{

class RegisterMapHelper
{
private:
    static RegisterMapHelper* instance;
    static uint32_t lock;

    RegistermapInterface* register_map = nullptr;

protected:
    RegisterMapHelper(RegistermapInterface* register_map);
    virtual ~RegisterMapHelper();

public:
    RegisterMapHelper(RegisterMapHelper& other) = delete;
    void operator=(const RegisterMapHelper& other) = delete;

    static RegisterMapHelper* get_instance(RegistermapInterface* register_map);

    uint8_t undo_register_write_change(uint8_t new_value, uint8_t written_bit_mask);

    void set_gyroscope_sensor_error(bool error);
    void set_accelerometer_sensor_error(bool error);
    void set_magnetometer_sensor_error(bool error);
    void set_barometer_sensor_error(bool error);
    bool has_sensor_error() const;
    void clear_sensor_errors();

    bool is_fusion_running();
    void set_sensor_fusion_running_status(bool running);
    void clear_fusion_control_start_stop_bit();

    bool is_temperature_data_read_by_host();
    bool is_pressure_data_read_by_host();
    bool is_gyroscope_data_read_by_host();
    bool is_accelerometer_data_read_by_host();
    bool is_magnetometer_data_read_by_host();
    bool is_quaternion_data_read_by_host();

    bool is_pressure_data_ready_flag();
    bool is_gyroscope_data_ready_flag();
    bool is_accelerometer_data_ready_flag();
    bool is_magnetometer_data_ready_flag();
    bool is_quaternion_data_ready_flag();
    register_fields::data_ready_t get_data_ready_flags();

    void set_pressure_data_ready_flag(bool value);
    void set_gyroscope_data_ready_flag(bool value);
    void set_accelerometer_data_ready_flag(bool value);
    void set_magnetometer_data_ready_flag(bool value);
    void set_quaternion_data_ready_flag(bool value);
    void clear_data_ready_flags();

    void write_pressure_data(const sensor::pressure_t& pressure_data);
    void write_temperature_data(const sensor::temperature_t& temperature_data);
    void write_gyroscope_data(const FusionVector& gyroscope_data);
    void write_accelerometer_data(const FusionVector& accelerometer_data);
    void write_magnetometer_data(const FusionVector& magnetometer_data);
    void write_quaternion_data(const FusionQuaternion& quaternion_data);

    void set_data_registers_as_read();
};

} // namespace storage
