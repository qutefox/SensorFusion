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

    RegisterMapInterface* register_map = nullptr;

protected:
    RegisterMapHelper(RegisterMapInterface* register_map);
    virtual ~RegisterMapHelper();

public:
    RegisterMapHelper(RegisterMapHelper& other) = delete;
    void operator=(const RegisterMapHelper& other) = delete;

    static RegisterMapHelper* get_instance(RegisterMapInterface* register_map);

    void set_gyroscope_sensor_error(bool error);
    void set_accelerometer_sensor_error(bool error);
    void set_magnetometer_sensor_error(bool error);
    void set_barometer_sensor_error(bool error);
    bool has_sensor_error() const;
    void clear_sensor_errors();

    void clear_control_bits();

    bool is_fusion_running();
    void set_fusion_running_status(bool running);

    bool is_quaternion_data_read_by_host();
    bool is_euler_data_read_by_host();
    bool is_earth_data_read_by_host();
    bool is_gyroscope_data_read_by_host();
    bool is_accelerometer_data_read_by_host();
    bool is_magnetometer_data_read_by_host();
    bool is_pressure_data_read_by_host();
    bool is_temperature_data_read_by_host();
    
    void set_quaternion_data_ready_flag(bool value);
    void set_euler_data_ready_flag(bool value);
    void set_earth_data_ready_flag(bool value);
    void set_gyroscope_data_ready_flag(bool value);
    void set_accelerometer_data_ready_flag(bool value);
    void set_magnetometer_data_ready_flag(bool value);
    void set_pressure_data_ready_flag(bool value);
    void set_temperature_data_ready_flag(bool value);
    void clear_data_ready_flags();

    void write_quaternion_data(const FusionQuaternion& quaternion_data);
    void write_euler_data(const FusionEuler& euler_data);
    void write_earth_data(const FusionVector& earth_data);
    void write_gyroscope_data(const FusionVector& gyroscope_data);
    void write_accelerometer_data(const FusionVector& accelerometer_data);
    void write_magnetometer_data(const FusionVector& magnetometer_data);
    void write_pressure_data(const sensor::pressure_t& pressure_data);
    void write_temperature_data(const sensor::temperature_t& temperature_data);
    

    void set_data_registers_as_read();

    bool is_calibration_uploading();
    void set_calibration_data_write_enabled(bool write_enabled);
    void set_calibration_uploading_status(bool uploading);
};

} // namespace storage
