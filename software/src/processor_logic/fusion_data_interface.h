#pragma once

#include "src/Fusion/Fusion/FusionMath.h"
#include "src/sensor/sensor_types.h"

class FusionDataInterface
{
public:
    FusionDataInterface() { }
    virtual ~FusionDataInterface() { }

    virtual void reset() = 0;
    virtual void update_register_map() = 0;
    virtual void update_calibration_data() = 0;
    virtual void set_use_calibration_data(bool use_calibration_data) = 0;

    virtual void update_gyroscope(const FusionVector& gyroscope_data) = 0;
    virtual void update_accelerometer(const FusionVector& accelerometer_data) = 0;
    virtual void update_magnetometer(const FusionVector& magnetometer_data) = 0;
    virtual void update_pressure(sensor::pressure_t pressure_data) = 0;
    virtual void update_temperature(sensor::temperature_t temperature_data) = 0;
    virtual void update_timestamp(sensor::timestamp_t timestamp_data) = 0;
};
