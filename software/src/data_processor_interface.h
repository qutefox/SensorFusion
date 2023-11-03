#pragma once

#include <stdint.h>

#include "src/io/output_pin.h"
#include "src/storage/register_interface.h"

class DataProcessorInterface
{
public:
    DataProcessorInterface() { }
    virtual ~DataProcessorInterface() { }

    int begin();

    virtual storage::MultiRegisterInterface<uint8_t, uint8_t>* get_register_map() const = 0;

    virtual void update_register_map() = 0;

    virtual void set_gyro_sensor_error(bool error) = 0;
    virtual void set_accel_sensor_error(bool error) = 0;
    virtual void set_mag_sensor_error(bool error) = 0;
    virtual void set_baro_sensor_error(bool error) = 0;
    virtual bool has_sensor_error() const = 0;

    virtual void set_baro_data(int32_t pressure, int16_t temperature) = 0;
};
