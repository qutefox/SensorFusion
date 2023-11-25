#pragma once

#include <stdint.h>

#include "src/processor_logic/data_processor_interface.h"
#include "src/storage/register_map_interface.h"
#include "src/timer/timer_interface.h"
#include "src/storage/register_fields.h"
#include "src/storage/register_map_helper.h"
#include "src/sensor_fusion_board.h"
#include "src/Fusion/Fusion/Fusion.h"

class DataProcessor : public DataProcessorInterface
{
private:
    static DataProcessor* instance;
    static uint32_t lock;

    RegistermapInterface* register_map = nullptr;
    storage::RegisterMapHelper* register_map_helper = nullptr;
    SensorFusionBoard* board = nullptr;
    timer::TimerInterface* continuous_timer = nullptr;
    // timer::TimerInterface* oneshot_timer = nullptr;

    FusionAhrs ahrs;
    FusionQuaternion quaternion_fqvect = {0.0f, 0.0f, 0.0f, 0.0f};
    FusionVector gyroscope_fvect = {0.0f, 0.0f, 0.0f};
    FusionVector accelerometer_fvect = {0.0f, 0.0f, 0.0f};
    FusionVector magnetometer_fvect = {0.0f, 0.0f, 0.0f};

    sensor::pressure_t pressure = {0};
    sensor::temperature_t temperature = {0};
    sensor::timestamp_t timestamp = {0};
    sensor::timestamp_t previous_timestamp = {0};

    bool has_new_pressure_data = false;
    bool has_new_temperature_data = false;
    bool has_new_gyroscope_data = false;
    bool has_new_accelerometer_data = false;
    bool has_new_magnetometer_data = false;
    bool has_new_quaternion_data = false;

    // void update_data_ready_flags_and_write_new_data();

    void handle_register_writes();
    void handle_register_reads();

    void handle_change_in_board_control_register();
    void handle_change_in_fusion_control_register();
    void handle_change_in_sensor_control();
    void handle_change_in_sensor_calibration_control_register();

    float get_time_delta();

    bool can_host_control_led();
    bool start_sensor_fusion();
    void stop_sensor_fusion();

    void update_led(storage::register_fields::board_control_t board_control_reg);
    void update_fusion_start_stop(storage::register_fields::fusion_control_t fusion_control_reg);
    void update_sensor_control(storage::register_fields::sensor_control_t sensor_control_reg, uint8_t written_bit_mask);

protected:
    DataProcessor();
    virtual ~DataProcessor();

public:
    DataProcessor(DataProcessor& other) = delete;
    void operator=(const DataProcessor& other) = delete;

    static DataProcessor* get_instance();

    virtual void update_register_map() override;
    virtual void update_fusion() override;

    virtual void update_gyroscope_fusion_vector(const FusionVector& gyroscope_data) override;
    virtual void update_accelerometer_fusion_vector(const FusionVector& accelerometer_data) override;
    virtual void update_magnetometer_fusion_vector(const FusionVector& magnetometer_data) override;
    virtual void update_pressure(sensor::pressure_t pressure_data) override;
    virtual void update_temperature(sensor::temperature_t temperature_data) override;
    virtual void update_timestamp(sensor::timestamp_t timestamp_data) override;

    virtual void set_gyroscope_sensor_error(bool error) override;
    virtual void set_accelerometer_sensor_error(bool error) override;
    virtual void set_magnetometer_sensor_error(bool error) override;
    virtual void set_barometer_sensor_error(bool error) override;
    virtual bool has_sensor_error() const override;
};
