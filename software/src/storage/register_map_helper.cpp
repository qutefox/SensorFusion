#include "register_map_helper.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

using namespace storage;

RegisterMapHelper* RegisterMapHelper::instance = nullptr;
uint32_t RegisterMapHelper::lock = 0;

RegisterMapHelper::RegisterMapHelper(RegisterMapInterface* _register_map)
    : register_map{ _register_map }
{
    
}

RegisterMapHelper::~RegisterMapHelper()
{

}

RegisterMapHelper* RegisterMapHelper::get_instance(RegisterMapInterface* register_map)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new RegisterMapHelper(register_map);
    }
    MXC_FreeLock(&lock);
    return instance;
}

void RegisterMapHelper::set_gyroscope_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_status_register()->set_bits(STATUS_REGISTER_GYRO_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_GYRO_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_accelerometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_status_register()->set_bits(STATUS_REGISTER_ACCEL_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_ACCEL_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_magnetometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_status_register()->set_bits(STATUS_REGISTER_MAG_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_MAG_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_barometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_status_register()->set_bits(STATUS_REGISTER_BARO_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_BARO_ERROR_MASK, false, false);
    }
}

bool RegisterMapHelper::has_sensor_error() const
{
    uint8_t reg;
    register_map->get_status_register()->read(reg, false);
    return reg & STATUS_REGISTER_ERRORS_MASK;
}

void RegisterMapHelper::clear_sensor_errors()
{
    register_map->get_status_register()->clear_bits(STATUS_REGISTER_ERRORS_MASK, false, false);
}

bool RegisterMapHelper::is_fusion_running()
{
    storage::registers::registers_u reg;
    register_map->get_status_register()->read(reg.byte, false);
    return reg.status.fusion_running == 1;
}

void RegisterMapHelper::set_fusion_running_status(bool running)
{
    if (running)
    {
        register_map->get_powermode_register()->set_write_enabled(false);
        register_map->get_status_register()->set_bits(STATUS_REGISTER_FUSION_RUNNING_MASK, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_FUSION_RUNNING_MASK, false);
        register_map->get_powermode_register()->set_write_enabled(true);
    }
}

bool RegisterMapHelper::is_quaternion_data_read_by_host()
{
    return register_map->get_quaternion_data_register()->is_read();
}

bool RegisterMapHelper::is_euler_data_read_by_host()
{
    return register_map->get_euler_data_register()->is_read();
}

bool RegisterMapHelper::is_earth_data_read_by_host()
{
    return register_map->get_earth_data_register()->is_read();
}

bool RegisterMapHelper::is_gyroscope_data_read_by_host()
{
    return register_map->get_gyroscope_data_register()->is_read();
}

bool RegisterMapHelper::is_accelerometer_data_read_by_host()
{
    return register_map->get_accelerometer_data_register()->is_read();
}

bool RegisterMapHelper::is_magnetometer_data_read_by_host()
{
    return register_map->get_magnetometer_data_register()->is_read();
}

bool RegisterMapHelper::is_pressure_data_read_by_host()
{
    return register_map->get_pressure_data_register()->is_read();
}

bool RegisterMapHelper::is_temperature_data_read_by_host()
{
    return register_map->get_temperature_data_register()->is_read();
}

void RegisterMapHelper::set_quaternion_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_QUAT_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_QUAT_MASK, false, false);
    }
}

void RegisterMapHelper::set_euler_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_EULER_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_EULER_MASK, false, false);
    }
}

void RegisterMapHelper::set_earth_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_EARTH_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_EARTH_MASK, false, false);
    }
}

void RegisterMapHelper::set_gyroscope_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_GYRO_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_GYRO_MASK, false, false);
    }
}

void RegisterMapHelper::set_accelerometer_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_ACCEL_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_ACCEL_MASK, false, false);
    }
}

void RegisterMapHelper::set_magnetometer_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_MAG_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_MAG_MASK, false, false);
    }
}

void RegisterMapHelper::set_pressure_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_BARO_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_BARO_MASK, false, false);
    }
}

void RegisterMapHelper::set_temperature_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_REGISTER_TEMP_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_REGISTER_TEMP_MASK, false, false);
    }
}

void RegisterMapHelper::clear_data_ready_flags()
{
    register_map->get_data_ready_register()->write(0x00, false, false);
}

void RegisterMapHelper::write_quaternion_data(const FusionQuaternion& quaternion_data)
{
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(quaternion_data.array);
    register_map->get_quaternion_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_euler_data(const FusionEuler& euler_data)
{
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(euler_data.array);
    register_map->get_euler_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_earth_data(const FusionVector& earth_data)
{
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(earth_data.array);
    register_map->get_earth_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_gyroscope_data(const FusionVector& gyroscope_data)
{   
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(gyroscope_data.array);
    register_map->get_gyroscope_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_accelerometer_data(const FusionVector& accelerometer_data)
{
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(accelerometer_data.array);
    register_map->get_accelerometer_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_magnetometer_data(const FusionVector& magnetometer_data)
{
    const uint8_t* array_data = reinterpret_cast<const uint8_t*>(magnetometer_data.array);
    register_map->get_magnetometer_data_register()->write(array_data, false, false);
}

void RegisterMapHelper::write_pressure_data(const sensor::pressure_t& pressure_data)
{
    // We need to shift by 1 because the data is 3 byte long, but we store it as uint32_t which has 4 bytes.
    // In order to pass here the last 3 relevant bytes we skip over the first byte.
    register_map->get_pressure_data_register()->write(pressure_data.u8bit+1, false, false);
}

void RegisterMapHelper::write_temperature_data(const sensor::temperature_t& temperature_data)
{
    register_map->get_temperature_data_register()->write(temperature_data.u8bit, false, false);
}

void RegisterMapHelper::set_data_registers_as_read()
{
    register_map->get_quaternion_data_register()->set_read();
    register_map->get_euler_data_register()->set_read();
    register_map->get_earth_data_register()->set_read();
    register_map->get_gyroscope_data_register()->set_read();
    register_map->get_accelerometer_data_register()->set_read();
    register_map->get_magnetometer_data_register()->set_read();
    register_map->get_pressure_data_register()->set_read();
    register_map->get_temperature_data_register()->set_read();
}

bool RegisterMapHelper::is_calibration_uploading()
{
    registers::registers_u reg;
    register_map->get_status_register()->read(reg.byte, false);
    return reg.status.calibration_uploading == 1;
}

void RegisterMapHelper::set_calibration_data_write_enabled(bool write_enabled)
{
    register_map->get_gyroscope_misalignment_register()->set_write_enabled(write_enabled);
    register_map->get_gyroscope_sensitivity_register()->set_write_enabled(write_enabled);
    register_map->get_gyroscope_offset_register()->set_write_enabled(write_enabled);

    register_map->get_accelerometer_misalignment_register()->set_write_enabled(write_enabled);
    register_map->get_accelerometer_sensitivity_register()->set_write_enabled(write_enabled);
    register_map->get_accelerometer_offset_register()->set_write_enabled(write_enabled);

    register_map->get_soft_iron_matrix_register()->set_write_enabled(write_enabled);
    register_map->get_hard_iron_offset_register()->set_write_enabled(write_enabled);
}

void RegisterMapHelper::set_calibration_uploading_status(bool uploading)
{
    if (uploading)
    {
        register_map->get_status_register()->set_bits(STATUS_REGISTER_CALIBRATION_UPLOADING_MASK, false);
    }
    else
    {
        register_map->get_status_register()->clear_bits(STATUS_REGISTER_CALIBRATION_UPLOADING_MASK, false);
    }
}

void RegisterMapHelper::clear_control_bits()
{
    register_map->get_control_register()->clear_bits(
        CONTROL_REGISTER_FUSION_START_MASK |
        CONTROL_REGISTER_FUSION_STOP_MASK |
        CONTROL_REGISTER_CALIBRATION_START_MASK |
        CONTROL_REGISTER_CALIBRATION_STOP_MASK |
        CONTROL_REGISTER_CALIBRATION_CANCEL_MASK |
        CONTROL_REGISTER_CALIBRATION_RESET_MASK, false, false);
}
