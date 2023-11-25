#include "src/storage/register_map_helper.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

using namespace storage;

RegisterMapHelper* RegisterMapHelper::instance = nullptr;
uint32_t RegisterMapHelper::lock = 0;

RegisterMapHelper::RegisterMapHelper(RegistermapInterface* _register_map)
    : register_map{ _register_map }
{
    
}

RegisterMapHelper::~RegisterMapHelper()
{

}

RegisterMapHelper* RegisterMapHelper::get_instance(RegistermapInterface* register_map)
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new RegisterMapHelper(register_map);
    }
    MXC_FreeLock(&lock);
    return instance;
}

uint8_t RegisterMapHelper::undo_register_write_change(uint8_t new_value, uint8_t written_bit_mask)
{
    uint8_t unchanged_bits = ((~written_bit_mask) & new_value);
    uint8_t reverted_bits = ((~(new_value & written_bit_mask)) & written_bit_mask);
    return unchanged_bits | reverted_bits;
}

void RegisterMapHelper::set_gyroscope_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_sensor_status_register()->set_bits(SENSOR_STATUS_GYRO_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_sensor_status_register()->clear_bits(SENSOR_STATUS_GYRO_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_accelerometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_sensor_status_register()->set_bits(SENSOR_STATUS_ACCEL_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_sensor_status_register()->clear_bits(SENSOR_STATUS_ACCEL_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_magnetometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_sensor_status_register()->set_bits(SENSOR_STATUS_MAG_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_sensor_status_register()->clear_bits(SENSOR_STATUS_MAG_ERROR_MASK, false, false);
    }
}

void RegisterMapHelper::set_barometer_sensor_error(bool error)
{
    if (error)
    {
        register_map->get_sensor_status_register()->set_bits(SENSOR_STATUS_BARO_ERROR_MASK, false, false);
    }
    else
    {
        register_map->get_sensor_status_register()->clear_bits(SENSOR_STATUS_BARO_ERROR_MASK, false, false);
    }
}

bool RegisterMapHelper::has_sensor_error() const
{
    uint8_t reg;
    register_map->get_sensor_status_register()->read(reg, false);
    return reg & SENSOR_STATUS_ERROR_MASK;
}

void RegisterMapHelper::clear_sensor_errors()
{
    register_map->get_sensor_status_register()->clear_bits(SENSOR_STATUS_ERROR_MASK, false, false);
}

bool RegisterMapHelper::is_fusion_running()
{
    storage::register_fields::register_types_u reg;
    register_map->get_fusion_status_register()->read(reg.byte, false);
    return reg.fusion_status.running == 1;
}

void RegisterMapHelper::set_sensor_fusion_running_status(bool running)
{
    if (running)
    {
        register_map->get_fusion_status_register()->set_bits(FUSION_STATUS_RUNNING_MASK, false);
    }
    else
    {
        register_map->get_fusion_status_register()->clear_bits(FUSION_STATUS_RUNNING_MASK, false);
    }
}

void RegisterMapHelper::clear_fusion_control_start_stop_bit()
{
    register_map->get_fusion_control_register()->clear_bits(FUSION_CONTROL_START_STOP_MASK, false, false);
}

bool RegisterMapHelper::is_temperature_data_read_by_host()
{
    return register_map->get_temperature_registers()->is_read();
}

bool RegisterMapHelper::is_pressure_data_read_by_host()
{
    return register_map->get_pressure_registers()->is_read();
}

bool RegisterMapHelper::is_gyroscope_data_read_by_host()
{
    return register_map->get_gyroscope_fusion_registers()->is_read();
}

bool RegisterMapHelper::is_accelerometer_data_read_by_host()
{
    return register_map->get_accelerometer_fusion_registers()->is_read();
}

bool RegisterMapHelper::is_magnetometer_data_read_by_host()
{
    return register_map->get_magnetometer_fusion_registers()->is_read();
}

bool RegisterMapHelper::is_quaternion_data_read_by_host()
{
    return register_map->get_quaternion_fusion_registers()->is_read();
}

bool RegisterMapHelper::is_pressure_data_ready_flag()
{
    uint8_t reg = 0;
    register_map->get_data_ready_register()->read(reg, false);
    return reg & DATA_READY_BARO_MASK;
}

bool RegisterMapHelper::is_gyroscope_data_ready_flag()
{
    uint8_t reg = 0;
    register_map->get_data_ready_register()->read(reg, false);
    return reg & DATA_READY_GYRO_MASK;
}

bool RegisterMapHelper::is_accelerometer_data_ready_flag()
{
    uint8_t reg = 0;
    register_map->get_data_ready_register()->read(reg, false);
    return reg & DATA_READY_ACCEL_MASK;
}

bool RegisterMapHelper::is_magnetometer_data_ready_flag()
{
    uint8_t reg = 0;
    register_map->get_data_ready_register()->read(reg, false);
    return reg & DATA_READY_MAG_MASK;
}

bool RegisterMapHelper::is_quaternion_data_ready_flag()
{
    uint8_t reg = 0;
    register_map->get_data_ready_register()->read(reg, false);
    return reg & DATA_READY_QUAT_MASK;
}

register_fields::data_ready_t RegisterMapHelper::get_data_ready_flags()
{
    register_fields::register_types_u reg;
    register_map->get_data_ready_register()->read(reg.byte, false);
    return reg.data_ready;
}

void RegisterMapHelper::set_pressure_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_BARO_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_BARO_MASK, false, false);
    }
}

void RegisterMapHelper::set_gyroscope_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_GYRO_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_GYRO_MASK, false, false);
    }
}

void RegisterMapHelper::set_accelerometer_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_ACCEL_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_ACCEL_MASK, false, false);
    }
}

void RegisterMapHelper::set_magnetometer_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_MAG_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_MAG_MASK, false, false);
    }
}

void RegisterMapHelper::set_quaternion_data_ready_flag(bool value)
{
    if (value)
    {
        register_map->get_data_ready_register()->set_bits(DATA_READY_QUAT_MASK, false, false);
    }
    else
    {
        register_map->get_data_ready_register()->clear_bits(DATA_READY_QUAT_MASK, false, false);
    }
}

void RegisterMapHelper::clear_data_ready_flags()
{
    register_map->get_data_ready_register()->write(0x00, false, false);
}
