#pragma once

#include <stdint.h>

#include "src/data_processor_interface.h"
#include "src/sensor_fusion_board.h"

class DataProcessor : public DataProcessorInterface
{
private:
    bool init_done = false;
    static DataProcessor* instance;
    static uint32_t lock;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* register_map;
    SensorFusionBoard* board = nullptr;

    storage::RegisterInterface<uint8_t, uint8_t>* sensor_error_register;
    storage::RegisterInterface<uint8_t, uint8_t>* data_ready_register;
    storage::RegisterInterface<uint8_t, uint8_t>* led_register;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* quat_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* baro_pressure_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* baro_temperature_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* inertial_gyro_axis_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* inertial_accel_axis_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* inertial_temperature_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* mag_axis_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* mag_temperature_registers;

    bool has_new_baro_data = false;
    sensor::pressure_t baro_pressure_data;
    sensor::temperature_t baro_temperature_data;

    bool has_new_inertial_data = false;
    sensor::axis3bit16_t inertial_gyro_data;
    sensor::axis3bit16_t inertial_accel_data;
    sensor::temperature_t inertial_temperature_data;

    bool has_new_mag_data = false;
    sensor::axis3bit16_t mag_data;
    sensor::temperature_t mag_temperature_data;

    void update_data_ready_flags_and_write_new_data();

protected:
    DataProcessor();
    virtual ~DataProcessor();

public:
    DataProcessor(DataProcessor& other) = delete;
    void operator=(const DataProcessor& other) = delete;

    static DataProcessor* get_instance();

    int begin();

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_register_map() const override;

    virtual void update_register_map() override;

    virtual void set_gyro_sensor_error(bool error) override;
    virtual void set_accel_sensor_error(bool error) override;
    virtual void set_mag_sensor_error(bool error) override;
    virtual void set_baro_sensor_error(bool error) override;
    virtual bool has_sensor_error() const override;

    virtual void update_baro_data(const sensor::pressure_t& pressure, const sensor::temperature_t& temperature) override;
    virtual void update_inertial_data(const sensor::axis3bit16_t& gyro, const sensor::axis3bit16_t& accel, const sensor::temperature_t& temperature) override;
    virtual void update_mag_data(const sensor::axis3bit16_t& mag, const sensor::temperature_t& temperature) override;
};
