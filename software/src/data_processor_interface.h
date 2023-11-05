#pragma once

#include <stdint.h>

#include "src/io/output_pin.h"
#include "src/storage/register_interface.h"
#include "src/sensor/sensor_types.h"

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

    virtual void update_baro_data(const sensor::pressure_t& pressure, const sensor::temperature_t& temperature) = 0;
    virtual void update_inertial_data(const sensor::axis3bit16_t& gyro, const sensor::axis3bit16_t& accel, const sensor::temperature_t& temperature) = 0;
    virtual void update_mag_data(const sensor::axis3bit16_t& mag, const sensor::temperature_t& temperature) = 0;

};
