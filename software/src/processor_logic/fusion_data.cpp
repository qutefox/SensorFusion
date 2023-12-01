#include "fusion_data.h"

#include "mxc_lock.h"

#include "src/storage/register_map.h"
#include "src/sensor_fusion_board.h"
#include "src/sensor/lsm6dsm.h"
#include "src/debug_print.h"

FusionData* FusionData::instance = nullptr;
uint32_t FusionData::lock = 0;

FusionData::FusionData()
    : register_map{ storage::RegisterMap::get_instance() }
    , register_map_helper{ storage::RegisterMapHelper::get_instance(register_map) }
    , calibration_data{ CalibrationData::get_instance() }
    , quaternion_fqvect{ 0.0f, 0.0f, 0.0f, 0.0f }
    , euler{ 0.0f, 0.0f, 0.0f }
    , earth{ 0.0f, 0.0f, 0.0f }
    , gyroscope_fvect{ 0.0f, 0.0f, 0.0f }
    , accelerometer_fvect{ 0.0f, 0.0f, 0.0f }
    , magnetometer_fvect{ 0.0f, 0.0f, 0.0f }
    , pressure{ 0 }
    , temperature{ 0 }
    , timestamp{ 0 }
    , previous_timestamp{ 0 }
    , has_new_quaternion_data{ false }
    , has_new_euler_data{ false }
    , has_new_earth_data{ false }
    , has_new_pressure_data{ false }
    , has_new_temperature_data{ false }
    , has_new_gyroscope_data{ false }
    , has_new_accelerometer_data{ false }
    , has_new_magnetometer_data{ false }
    , use_calibration_data{ false }
{
    FusionAhrsInitialise(&ahrs);

    if (CONTROL_REGISTER_DEFAULT_VALUE & CONTROL_REGISTER_CALIBRATION_ACTIVE_MASK)
    {
        use_calibration_data = true;
    }

    update_calibration_data();

    // Cannot call reset here, because the SensorBase class constucts the fusion data
    // before the SensorFusionBoard class does. We use the board class to get the sensor pointer
    // to read the data rate (board->inertial_sensor->get_data_rate()). At this point the sensor
    // pointers are not initialised and this causes a nullpointer exception.
    // The Controller class wil call reset when starting the fusion.
    // reset();
}

FusionData::~FusionData()
{
    FusionAhrsReset(&ahrs);
}

FusionData* FusionData::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new FusionData();
    }
    MXC_FreeLock(&lock);
    return instance;
}

void FusionData::reset()
{
    uint16_t gyro_sample_rate = 100;

    sensor::SensorInterface* sensor = nullptr;
    SensorFusionBoard* board = SensorFusionBoard::get_instance();
    if (board != nullptr)
    {
        sensor = board->get_inertial_sensor();
    }
    
    if (sensor != nullptr)
    {
        gyro_sample_rate = sensor->get_sample_rate_in_hz(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO);
        if (gyro_sample_rate == 0) gyro_sample_rate = 100;
    }

    FusionOffsetInitialise(&ahrs_offset, gyro_sample_rate);

    // Set AHRS algorithm settings.
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f, // gyroscope range in degrees/s
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * gyro_sample_rate, // 5 seconds
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    FusionAhrsReset(&ahrs);
    
    has_new_quaternion_data = false;
    has_new_euler_data = false;
    has_new_earth_data = false;
    has_new_pressure_data = false;
    has_new_temperature_data = false;
    has_new_gyroscope_data = false;
    has_new_accelerometer_data = false;
    has_new_magnetometer_data = false;
}

void FusionData::update_register_map()
{
    if (register_map_helper->is_quaternion_data_read_by_host())
    {
        if (has_new_quaternion_data)
        {
            register_map_helper->write_quaternion_data(quaternion_fqvect);
            has_new_quaternion_data = false;
            register_map_helper->set_quaternion_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_quaternion_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_euler_data_read_by_host())
    {
        if (has_new_euler_data)
        {
            register_map_helper->write_euler_data(euler);
            has_new_euler_data = false;
            register_map_helper->set_euler_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_euler_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_earth_data_read_by_host())
    {
        if (has_new_earth_data)
        {
            register_map_helper->write_earth_data(earth);
            has_new_earth_data = false;
            register_map_helper->set_earth_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_earth_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_gyroscope_data_read_by_host())
    {
        if (has_new_gyroscope_data)
        {
            register_map_helper->write_gyroscope_data(gyroscope_fvect);
            has_new_gyroscope_data = false;
            register_map_helper->set_gyroscope_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_gyroscope_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_accelerometer_data_read_by_host())
    {
        if (has_new_accelerometer_data)
        {
            register_map_helper->write_accelerometer_data(accelerometer_fvect);
            has_new_accelerometer_data = false;
            register_map_helper->set_accelerometer_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_accelerometer_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_magnetometer_data_read_by_host())
    {
        if (has_new_magnetometer_data)
        {
            register_map_helper->write_magnetometer_data(magnetometer_fvect);
            has_new_magnetometer_data = false;
            register_map_helper->set_magnetometer_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_magnetometer_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_pressure_data_read_by_host())
    {
        if (has_new_pressure_data)
        {
            register_map_helper->write_pressure_data(pressure);
            has_new_pressure_data = false;
            register_map_helper->set_pressure_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_pressure_data_ready_flag(false);
        }
    }

    if (register_map_helper->is_temperature_data_read_by_host())
    {
        if (has_new_temperature_data)
        {
            register_map_helper->write_temperature_data(temperature);
            has_new_temperature_data = false;
            register_map_helper->set_temperature_data_ready_flag(true);
        }
        else
        {
            register_map_helper->set_temperature_data_ready_flag(false);
        }
    }
}

void FusionData::set_use_calibration_data(bool _use_calibration_data)
{
    use_calibration_data = _use_calibration_data;
}

void FusionData::update_calibration_data()
{
    gyroscope_misalignment = calibration_data->get_gyroscope_misalignment();
    gyroscope_sensitivity = calibration_data->get_gyroscope_sensitivity();
    gyroscope_offset = calibration_data->get_gyroscope_offset();

    accelerometer_misalignment = calibration_data->get_accelerometer_misalignment();
    accelerometer_sensitivity = calibration_data->get_accelerometer_sensitivity();
    accelerometer_offset = calibration_data->get_accelerometer_offset();

    soft_iron_matrix = calibration_data->get_soft_iron_matrix();
    hard_iron_offset = calibration_data->get_hard_iron_offset();
}

void FusionData::update_fusion()
{
    // This method should be called repeatedly each time new gyroscope data is available.

    // Update gyroscope AHRS algorithm.
    FusionAhrsUpdate(&ahrs, gyroscope_fvect, accelerometer_fvect, magnetometer_fvect, get_time_delta_in_seconds());
    
    quaternion_fqvect = FusionAhrsGetQuaternion(&ahrs);
    has_new_quaternion_data = true;
    euler =  FusionQuaternionToEuler(quaternion_fqvect);
    has_new_euler_data = true;
    earth = FusionAhrsGetEarthAcceleration(&ahrs);
    has_new_earth_data = true;
}

void FusionData::update_gyroscope(const FusionVector& gyroscope_data)
{
    gyroscope_fvect = gyroscope_data;
    if (use_calibration_data)
    {
        // Apply calibration.
        gyroscope_fvect = FusionCalibrationInertial(gyroscope_fvect, gyroscope_misalignment, gyroscope_sensitivity, gyroscope_offset);
        // Update gyroscope offset correction algorithm
        gyroscope_fvect = FusionOffsetUpdate(&ahrs_offset, gyroscope_fvect);
    }
    has_new_gyroscope_data = true;
    update_fusion();
}

void FusionData::update_accelerometer(const FusionVector& accelerometer_data)
{
    accelerometer_fvect = accelerometer_data;
    if (use_calibration_data)
    {
        // Apply calibration.
        accelerometer_fvect = FusionCalibrationInertial(accelerometer_fvect, accelerometer_misalignment, accelerometer_sensitivity, accelerometer_offset);
    }
    has_new_accelerometer_data = true;
}

void FusionData::update_magnetometer(const FusionVector& magnetometer_data)
{
    magnetometer_fvect = magnetometer_data;
    if (use_calibration_data)
    {
        // Apply calibration.
        magnetometer_fvect = FusionCalibrationMagnetic(magnetometer_fvect, soft_iron_matrix, hard_iron_offset);
    }
    has_new_magnetometer_data = true;
}

void FusionData::update_pressure(sensor::pressure_t pressure_data)
{
    pressure = pressure_data;
    has_new_pressure_data = true;
}

void FusionData::update_temperature(sensor::temperature_t temperature_data)
{
    temperature.i16bit = (temperature.i16bit/2) + (temperature_data.i16bit/2);
    has_new_temperature_data = true;
}

void FusionData::update_timestamp(sensor::timestamp_t timestamp_data)
{
    timestamp = timestamp_data;
}

float FusionData::get_time_delta_in_seconds() // in seconds
{
    // We multiply by 25 because the timestamp generation (in LSM6DSM) has 25 microsecond resolution set.
    uint32_t diff_microseconds = (timestamp.u32bit - previous_timestamp.u32bit) * 25;
    previous_timestamp = timestamp;
    return static_cast<float>(diff_microseconds) / 1000000; // Convert microsecond to second.
}
