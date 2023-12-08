#pragma once

#include "fusion_data_interface.h"

#include "calibration_data.h"
#include "src/storage/register_map_interface.h"
#include "src/storage/register_map_helper.h"
#include "src/Fusion/Fusion/Fusion.h"

class FusionData : public FusionDataInterface
{
private:
    static FusionData* instance;
    static uint32_t lock;

    storage::RegisterMapInterface* register_map = nullptr;
    storage::RegisterMapHelper* register_map_helper = nullptr;
    CalibrationData* calibration_data = nullptr;

    FusionOffset ahrs_offset;
    FusionAhrs ahrs;
    FusionQuaternion quaternion_fqvect = FUSION_IDENTITY_QUATERNION;
    FusionEuler euler = FUSION_EULER_ZERO;
    FusionVector earth = FUSION_VECTOR_ZERO;
    FusionVector gyroscope_fvect = FUSION_VECTOR_ZERO;
    FusionVector accelerometer_fvect = FUSION_VECTOR_ZERO;
    FusionVector magnetometer_fvect = FUSION_VECTOR_ZERO;

    sensor::pressure_t pressure = {0};
    sensor::temperature_t temperature = {0};
    sensor::timestamp_t timestamp = {0};
    sensor::timestamp_t previous_timestamp = {0};

    bool has_new_quaternion_data = false;
    bool has_new_euler_data = false;
    bool has_new_earth_data = false;
    bool has_new_pressure_data = false;
    bool has_new_temperature_data = false;
    bool has_new_gyroscope_data = false;
    bool has_new_accelerometer_data = false;
    bool has_new_magnetometer_data = false;
    bool use_calibration_data = true;

    FusionMatrix gyroscope_misalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector gyroscope_sensitivity = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscope_offset = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometer_misalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector accelerometer_sensitivity = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometer_offset = {0.0f, 0.0f, 0.0f};
    FusionMatrix soft_iron_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector hard_iron_offset = {0.0f, 0.0f, 0.0f};

    void update_fusion();
    float get_time_delta_in_seconds();

protected:
    FusionData();
    virtual ~FusionData();

public:
    FusionData(FusionData& other) = delete;
    void operator=(const FusionData& other) = delete;

    static FusionData* get_instance();

    virtual void reset() override;
    virtual void update_register_map() override;
    virtual void set_use_calibration_data(bool use_calibration_data) override;
    virtual void update_calibration_data() override;

    virtual void update_gyroscope(const FusionVector& gyroscope_data) override;
    virtual void update_accelerometer(const FusionVector& accelerometer_data) override;
    virtual void update_magnetometer(const FusionVector& magnetometer_data) override;
    virtual void update_pressure(sensor::pressure_t pressure_data) override;
    virtual void update_temperature(sensor::temperature_t temperature_data) override;
    virtual void update_timestamp(sensor::timestamp_t timestamp_data) override;
};
