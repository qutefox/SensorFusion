#include "data_processor.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/register.h"
#include "src/storage/register_fields.h"
#include "src/debug_print.h"

DataProcessor* DataProcessor::instance = nullptr;
uint32_t DataProcessor::lock = 0;

DataProcessor::DataProcessor()
    : init_done{ false }
    , board{ SensorFusionBoard::get_instance() }
{
    auto make_registers_with_read_flag =
        [](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            for (uint8_t i = 0 ; i < length ; ++i)
            {
                registers[i] = new storage::RegisterWithReadFlag<uint8_t, uint8_t>(0x00, 0x00);
            }
        };

    sensor_error_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);
    data_ready_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);
    led_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0xFF, 0x00);
    baro_pressure_registers = new storage::MultiRegister<uint8_t, uint8_t>(3, true, make_registers_with_read_flag);
    baro_temperature_registers = new storage::MultiRegister<uint8_t, uint8_t>(2, true, make_registers_with_read_flag);

    register_map = new storage::MultiRegister<uint8_t, uint8_t>(8, false,
        [this](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            uint8_t idx = 0;
            registers[idx++] = sensor_error_register;
            registers[idx++] = data_ready_register;
            registers[idx++] = led_register;
            registers[idx++] = baro_pressure_registers;
            registers[idx++] = baro_pressure_registers->get_register(1);
            registers[idx++] = baro_pressure_registers->get_register(2);
            registers[idx++] = baro_temperature_registers;
            registers[idx++] = baro_temperature_registers->get_register(1);
        }
    );
}

DataProcessor::~DataProcessor()
{
    delete register_map;
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
    init_done = true;
    return E_NO_ERROR;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* DataProcessor::get_register_map() const
{
    return register_map;
}

void DataProcessor::update_register_map()
{
    storage::register_fields::register_types_u reg;

    // Handle register reads.
    data_ready_register->read(reg.byte, false);

    if (baro_pressure_registers->is_read() || baro_temperature_registers->is_read()) reg.data_ready.baro_data_ready = 0;
    // TODO: other sensors

    data_ready_register->write(reg.byte, false, false);

    // Handle register writes.
    uint8_t addr, written_bit_mask, new_value;
    while(register_map->get_next_written_and_changed_register(addr, written_bit_mask, new_value))
	{
        debug_print("register changed: addr=%02X, written_bit_mask=%02X, new_value=%02X.\n", addr, written_bit_mask, new_value);

        reg.byte = new_value;

        switch(addr)
        {
        case 2:
        
            board->get_led_pin()->set(reg.led.state);
            break;
        }
        // TODO: react to received data/change.
    }
}

void DataProcessor::set_baro_data(int32_t pressure, int16_t temperature)
{
    storage::register_fields::register_types_u reg;
    data_ready_register->read(reg.byte, false);

    if (!reg.data_ready.baro_data_ready || baro_pressure_registers->is_read() || baro_temperature_registers->is_read())
    {
        baro_pressure_registers->write(reinterpret_cast<uint8_t*>(&pressure), false, false);
        baro_temperature_registers->write(reinterpret_cast<uint8_t*>(&temperature), false, false);
        reg.data_ready.baro_data_ready = 1;
        data_ready_register->write(reg.byte, false, false);
    }

    // debug_print("set_baro_data done.\n");

    // TODO: update sensor fusion.
}

void DataProcessor::set_gyro_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.gyro_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_accel_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.accel_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_mag_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.mag_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

void DataProcessor::set_baro_sensor_error(bool error)
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    reg.sensor_errors.baro_error = error ? 1 : 0;
    sensor_error_register->write(reg.byte, false, false);
}

bool DataProcessor::has_sensor_error() const
{
    storage::register_fields::register_types_u reg;
    sensor_error_register->read(reg.byte, false);
    return reg.byte != 0;
}