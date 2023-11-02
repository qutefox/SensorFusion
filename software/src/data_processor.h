#pragma once

#include <stdint.h>

#include "src/storage/iregister.h"
#include "src/io/output_pin.h"

class DataProcessor
{
private:
    bool init_done = false;
    static DataProcessor* instance;
    static uint32_t lock;

    storage::IMultiRegister<uint8_t, uint8_t>* register_map;
    io::pin::Output* red_led = nullptr;

    storage::IRegister<uint8_t, uint8_t>* sensor_error_register;
    storage::IRegister<uint8_t, uint8_t>* data_ready_register;
    storage::IRegister<uint8_t, uint8_t>* led_register;
    storage::IMultiRegister<uint8_t, uint8_t>* baro_pressure_registers;
    storage::IMultiRegister<uint8_t, uint8_t>* baro_temperature_registers;

protected:
    DataProcessor();
    ~DataProcessor();

public:
    DataProcessor(DataProcessor& other) = delete;
    void operator=(const DataProcessor& other) = delete;

    static DataProcessor* get_instance();

    storage::IMultiRegister<uint8_t, uint8_t>* get_register_map() const;

    void set_red_led_instance(io::pin::Output* red_led);

    void update_register_map();

    void set_gyro_error(bool error);
    void set_accel_error(bool error);
    void set_mag_error(bool error);
    void set_baro_error(bool error);

    bool is_in_sensor_error() const;

    void set_baro_data(int32_t pressure, int16_t temperature);
};
