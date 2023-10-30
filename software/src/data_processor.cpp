#include "data_processor.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/register_map_reader_writer.h"
#include "src/debug_print.h"

DataProcessor* DataProcessor::instance = nullptr;
uint32_t DataProcessor::lock = 0;

DataProcessor::DataProcessor()
    : register_map_rw{ storage::RegisterMapReaderWriter::get_instance() }
    , red_led{ nullptr }
{

}

DataProcessor::~DataProcessor()
{

}

DataProcessor* DataProcessor::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new DataProcessor();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int DataProcessor::begin()
{
    return E_NO_ERROR;
}

void DataProcessor::set_red_led_instance(io::pin::Output* _red_led)
{
    red_led = _red_led;
}

int DataProcessor::handle_register_written_bits(uint8_t addr, uint8_t changed_bits, uint8_t new_value)
{
    // debug_print("register changed: addr=%02X, changed_bits=%02X, new_value=%02X.\n", addr, changed_bits, new_value);

    storage::register_types::register_types_u reg;
    reg.byte = new_value;

    switch (addr)
    {
    case SENSOR_FUSION_RED_LED:
        if (red_led != nullptr && !is_in_sensor_error())
        {
            // If we have a sensor error the LED is taken over by the system
            // and the user cannot set it's value.
            red_led->set(reg.red_led.state);
        }
        break;
    case SENSOR_FUSION_BARO_PRESSURE_XL:
    case SENSOR_FUSION_BARO_PRESSURE_L:
    case SENSOR_FUSION_BARO_PRESSURE_H:
        // Don't care. Data ready flag is set in the set_baro_pressure method.
        break;
    }

    return E_NO_ERROR;
}

int DataProcessor::handle_register_read(uint8_t addr)
{
    debug_print("register read: addr=%02X.\n", addr);

    storage::register_types::register_types_u reg;

    switch (addr)
    {
    case SENSOR_FUSION_DATA_READY:
        reg.byte = 0;
        break;
    }

    return E_NO_ERROR;
}

void DataProcessor::set_gyro_error(bool error)
{
    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_SENSOR_ERRORS, false);
    reg.sensor_errors.gyro_error = error ? 1 : 0;
    register_map_rw->write(SENSOR_FUSION_SENSOR_ERRORS, reg.byte, false);
}

void DataProcessor::set_accel_error(bool error)
{
    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_SENSOR_ERRORS, false);
    reg.sensor_errors.accel_error = error ? 1 : 0;
    register_map_rw->write(SENSOR_FUSION_SENSOR_ERRORS, reg.byte, false);
}

void DataProcessor::set_mag_error(bool error)
{
    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_SENSOR_ERRORS, false);
    reg.sensor_errors.mag_error = error ? 1 : 0;
    register_map_rw->write(SENSOR_FUSION_SENSOR_ERRORS, reg.byte, false);
}

void DataProcessor::set_baro_error(bool error)
{
    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_SENSOR_ERRORS, false);
    reg.sensor_errors.baro_error = error ? 1 : 0;
    register_map_rw->write(SENSOR_FUSION_SENSOR_ERRORS, reg.byte, false);
}

bool DataProcessor::is_in_sensor_error() const
{
    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_SENSOR_ERRORS, false);
    return reg.byte != 0;
}

void DataProcessor::set_baro_pressure(int32_t pressure)
{
    uint8_t bytes[3];
    bytes[0] = static_cast<uint8_t>(pressure & 0xFF);
    bytes[1] = static_cast<uint8_t>((pressure >> 8) & 0xFF);
    bytes[2] = static_cast<uint8_t>((pressure >> 16) & 0xFF);
    register_map_rw->write(SENSOR_FUSION_BARO_PRESSURE_XL, &bytes[0], 3, false);

    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_DATA_READY, false);
    reg.data_ready.baro_data_ready = 1;
    register_map_rw->write(SENSOR_FUSION_DATA_READY, reg.byte, false);
}

void DataProcessor::set_baro_temperature(int16_t temperature)
{
    uint8_t bytes[2];
    bytes[0] = static_cast<uint8_t>(temperature & 0xFF);
    bytes[1] = static_cast<uint8_t>((temperature >> 8) & 0xFF);
    register_map_rw->write(SENSOR_FUSION_BARO_TEMP_L, &bytes[0], 2, false);

    storage::register_types::register_types_u reg;
    reg.byte = register_map_rw->read(SENSOR_FUSION_DATA_READY, false);
    reg.data_ready.baro_data_ready = 1;
    register_map_rw->write(SENSOR_FUSION_DATA_READY, reg.byte, false);
}
