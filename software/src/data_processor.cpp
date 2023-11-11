#include "data_processor.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/register_fields.h"
#include "src/register_map.h"
#include "src/debug_print.h"

DataProcessor* DataProcessor::instance = nullptr;
uint32_t DataProcessor::lock = 0;

DataProcessor::DataProcessor()
    : register_map{ Registermap::get_instance() }
    , board{ SensorFusionBoard::get_instance() }
{   
    FusionAhrsInitialise(&ahrs);
}

DataProcessor::~DataProcessor()
{

}

DataProcessor* DataProcessor::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new DataProcessor();
    }
    MXC_FreeLock(&lock);
    return instance;
}

void DataProcessor::update_register_map()
{
    storage::register_fields::register_types_u reg;

    // Handle register reads.
    // TODO: update_data_ready_flags_and_write_new_data();

    // Handle register writes.
    uint8_t addr, written_bit_mask, new_value;
    while(register_map->get_addressable_base()->get_next_written_and_changed_register(addr, written_bit_mask, new_value))
	{
        // debug_print("register changed: addr=%02X, written_bit_mask=%02X, new_value=%02X.\n", addr, written_bit_mask, new_value);
        reg.byte = new_value;

        // TODO: react to received data/change.

        // switch(addr)
        // {
        // case 2:
        //     if (!has_sensor_error()) board->get_led_pin()->set(reg.led.state);
        //     break;
        // }
    }

    if (has_sensor_error()) board->get_led_pin()->write(true);
}

void DataProcessor::update_fusion()
{
    // This method should be called repeatedly each time new gyroscope data is available.

    // float deltaTime = get_time_delta();
    // FusionAhrsUpdate(&ahrs, gyroscope_fvect, accelerometer_fvect, magnetometer_fvect, deltaTime);
    // quaternion_fqvect = FusionAhrsGetQuaternion(&ahrs);
}

void DataProcessor::update_gyroscope_fusion_vector(const FusionVector& gyroscope_data)
{
    gyroscope_fvect = gyroscope_data;
    // update_fusion();
}

void DataProcessor::update_accelerometer_fusion_vector(const FusionVector& accelerometer_data)
{
    accelerometer_fvect = accelerometer_data;
}

void DataProcessor::update_magnetometer_fusion_vector(const FusionVector& magnetometer_data)
{
    magnetometer_fvect = magnetometer_data;
}

void DataProcessor::update_pressure(sensor::pressure_t pressure_data)
{
    pressure = pressure_data;
    debug_print("delta time: %f.\n", get_time_delta());
}

void DataProcessor::update_temperature(sensor::temperature_t temperature_data)
{
    temperature.i16bit = (temperature.i16bit/2) + (temperature_data.i16bit/2);
}

void DataProcessor::update_timestamp(sensor::timestamp_t timestamp_data)
{
    timestamp = timestamp_data;
}

float DataProcessor::get_time_delta()
{
    // Note: both values are unsigned.
    // If the result would be negative then we get a really big positive number.
    uint32_t diff_microseconds = (timestamp.u32bit - previous_timestamp.u32bit) * 25;
    previous_timestamp = timestamp;
    return static_cast<float>(diff_microseconds) / 1000000;
}

void DataProcessor::set_gyroscope_sensor_error(bool error)
{
    // TODO:
}

void DataProcessor::set_accelerometer_sensor_error(bool error)
{
    // TODO:
}

void DataProcessor::set_magnetometer_sensor_error(bool error)
{
    // TODO:
}

void DataProcessor::set_barometer_sensor_error(bool error)
{
    // TODO:
}

bool DataProcessor::has_sensor_error() const
{
    // TODO:
    return false;
}


/*
void DataProcessor::set_gyro_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.gyro_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_accel_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.accel_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_mag_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.mag_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_baro_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.baro_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

bool DataProcessor::has_sensor_error() const
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    return reg.byte != 0;
}
*/
