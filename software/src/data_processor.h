#pragma once

#include <stdint.h>

#include "data_processor_interface.h"
#include "sensor_fusion_board.h"

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
    storage::MultiRegisterInterface<uint8_t, uint8_t>* baro_pressure_registers;
    storage::MultiRegisterInterface<uint8_t, uint8_t>* baro_temperature_registers;

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

    virtual void set_baro_data(int32_t pressure, int16_t temperature) override;
};
