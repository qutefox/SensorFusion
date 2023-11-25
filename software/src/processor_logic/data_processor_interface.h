#pragma once

#include "src/Fusion/Fusion/FusionMath.h"
#include "src/sensor/sensor_types.h"

class DataProcessorInterface
{
public:
    DataProcessorInterface() { }
    virtual ~DataProcessorInterface() { }

    virtual void update_register_map() = 0;
    virtual void update_fusion() = 0;

    virtual void update_gyroscope_fusion_vector(const FusionVector& gyroscope_data) = 0;
    virtual void update_accelerometer_fusion_vector(const FusionVector& accelerometer_data) = 0;
    virtual void update_magnetometer_fusion_vector(const FusionVector& magnetometer_data) = 0;
    virtual void update_pressure(sensor::pressure_t pressure_data) = 0;
    virtual void update_temperature(sensor::temperature_t temperature_data) = 0;
    virtual void update_timestamp(sensor::timestamp_t timestamp_data) = 0;

    virtual void set_gyroscope_sensor_error(bool error) = 0;
    virtual void set_accelerometer_sensor_error(bool error) = 0;
    virtual void set_magnetometer_sensor_error(bool error) = 0;
    virtual void set_barometer_sensor_error(bool error) = 0;
    virtual bool has_sensor_error() const = 0;
};
