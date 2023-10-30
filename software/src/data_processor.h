#pragma once

#include <stdint.h>

#include "src/io/output_pin.h"

namespace storage
{

class RegisterMapReaderWriter; // Forward declare.

} // namespace storage

class DataProcessor
{
private:
    bool init_done = false;
    static DataProcessor* instance;
    static uint32_t lock;

    storage::RegisterMapReaderWriter* register_map_rw = nullptr;
    io::pin::Output* red_led = nullptr;

protected:
    DataProcessor();
    ~DataProcessor();

public:
    DataProcessor(DataProcessor& other) = delete;
    void operator=(const DataProcessor& other) = delete;

    static DataProcessor* get_instance();

    int begin();

    void set_red_led_instance(io::pin::Output* red_led);

    int handle_register_written_bits(uint8_t addr, uint8_t changed_bits, uint8_t new_value);
    int handle_register_read(uint8_t addr);

    void set_gyro_error(bool error);
    void set_accel_error(bool error);
    void set_mag_error(bool error);
    void set_baro_error(bool error);

    bool is_in_sensor_error() const;

    void set_baro_pressure(int32_t pressure);
    void set_baro_temperature(int16_t temperature);
};
