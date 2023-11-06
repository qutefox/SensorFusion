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
    , counter{ timer::Counter::get_instance() }
{
    create_register_map();
    
    FusionAhrsInitialise(&ahrs);

    counter->start_counter0(1000); // TODO: start/stop when fusion start/stops.
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

void DataProcessor::create_register_map()
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
    led_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0x01, 0x00);
    quat_registers = new storage::MultiRegister<uint8_t, uint8_t>(16, true, make_registers_with_read_flag);
    baro_pressure_registers = new storage::MultiRegister<uint8_t, uint8_t>(3, true, make_registers_with_read_flag);
    baro_temperature_registers = new storage::MultiRegister<uint8_t, uint8_t>(2, true, make_registers_with_read_flag);
    inertial_gyro_axis_registers = new storage::MultiRegister<uint8_t, uint8_t>(16, true, make_registers_with_read_flag);
    inertial_accel_axis_registers = new storage::MultiRegister<uint8_t, uint8_t>(16, true, make_registers_with_read_flag);
    inertial_temperature_registers = new storage::MultiRegister<uint8_t, uint8_t>(2, true, make_registers_with_read_flag);
    mag_axis_registers = new storage::MultiRegister<uint8_t, uint8_t>(16, true, make_registers_with_read_flag);
    mag_temperature_registers = new storage::MultiRegister<uint8_t, uint8_t>(2, true, make_registers_with_read_flag);

    register_map = new storage::MultiRegister<uint8_t, uint8_t>(8, false,
        [this](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            uint8_t idx = 0;
            registers[idx++] = sensor_error_register;
            registers[idx++] = data_ready_register;
            registers[idx++] = led_register;

            registers[idx++] = quat_registers;
            registers[idx++] = quat_registers->get_register(1);
            registers[idx++] = quat_registers->get_register(2);
            registers[idx++] = quat_registers->get_register(3);
            registers[idx++] = quat_registers->get_register(4);
            registers[idx++] = quat_registers->get_register(5);
            registers[idx++] = quat_registers->get_register(6);
            registers[idx++] = quat_registers->get_register(7);
            registers[idx++] = quat_registers->get_register(8);
            registers[idx++] = quat_registers->get_register(9);
            registers[idx++] = quat_registers->get_register(10);
            registers[idx++] = quat_registers->get_register(11);
            registers[idx++] = quat_registers->get_register(12);
            registers[idx++] = quat_registers->get_register(13);
            registers[idx++] = quat_registers->get_register(14);
            registers[idx++] = quat_registers->get_register(15);

            registers[idx++] = inertial_gyro_axis_registers;
            registers[idx++] = inertial_gyro_axis_registers->get_register(1);
            registers[idx++] = inertial_gyro_axis_registers->get_register(2);
            registers[idx++] = inertial_gyro_axis_registers->get_register(3);
            registers[idx++] = inertial_gyro_axis_registers->get_register(4);
            registers[idx++] = inertial_gyro_axis_registers->get_register(5);
            registers[idx++] = inertial_gyro_axis_registers->get_register(6);
            registers[idx++] = inertial_gyro_axis_registers->get_register(7);
            registers[idx++] = inertial_gyro_axis_registers->get_register(8);
            registers[idx++] = inertial_gyro_axis_registers->get_register(9);
            registers[idx++] = inertial_gyro_axis_registers->get_register(10);
            registers[idx++] = inertial_gyro_axis_registers->get_register(11);
            registers[idx++] = inertial_gyro_axis_registers->get_register(12);
            registers[idx++] = inertial_gyro_axis_registers->get_register(13);
            registers[idx++] = inertial_gyro_axis_registers->get_register(14);
            registers[idx++] = inertial_gyro_axis_registers->get_register(15);

            registers[idx++] = inertial_accel_axis_registers;
            registers[idx++] = inertial_accel_axis_registers->get_register(1);
            registers[idx++] = inertial_accel_axis_registers->get_register(2);
            registers[idx++] = inertial_accel_axis_registers->get_register(3);
            registers[idx++] = inertial_accel_axis_registers->get_register(4);
            registers[idx++] = inertial_accel_axis_registers->get_register(5);
            registers[idx++] = inertial_accel_axis_registers->get_register(6);
            registers[idx++] = inertial_accel_axis_registers->get_register(7);
            registers[idx++] = inertial_accel_axis_registers->get_register(8);
            registers[idx++] = inertial_accel_axis_registers->get_register(9);
            registers[idx++] = inertial_accel_axis_registers->get_register(10);
            registers[idx++] = inertial_accel_axis_registers->get_register(11);
            registers[idx++] = inertial_accel_axis_registers->get_register(12);
            registers[idx++] = inertial_accel_axis_registers->get_register(13);
            registers[idx++] = inertial_accel_axis_registers->get_register(14);
            registers[idx++] = inertial_accel_axis_registers->get_register(15);

            registers[idx++] = inertial_temperature_registers;
            registers[idx++] = inertial_temperature_registers->get_register(1);

            registers[idx++] = mag_axis_registers;
            registers[idx++] = mag_axis_registers->get_register(1);
            registers[idx++] = mag_axis_registers->get_register(2);
            registers[idx++] = mag_axis_registers->get_register(3);
            registers[idx++] = mag_axis_registers->get_register(4);
            registers[idx++] = mag_axis_registers->get_register(5);
            registers[idx++] = mag_axis_registers->get_register(6);
            registers[idx++] = mag_axis_registers->get_register(7);
            registers[idx++] = mag_axis_registers->get_register(8);
            registers[idx++] = mag_axis_registers->get_register(9);
            registers[idx++] = mag_axis_registers->get_register(10);
            registers[idx++] = mag_axis_registers->get_register(11);
            registers[idx++] = mag_axis_registers->get_register(12);
            registers[idx++] = mag_axis_registers->get_register(13);
            registers[idx++] = mag_axis_registers->get_register(14);
            registers[idx++] = mag_axis_registers->get_register(15);

            registers[idx++] = mag_temperature_registers;
            registers[idx++] = mag_temperature_registers->get_register(1);

            registers[idx++] = baro_pressure_registers;
            registers[idx++] = baro_pressure_registers->get_register(1);
            registers[idx++] = baro_pressure_registers->get_register(2);

            registers[idx++] = baro_temperature_registers;
            registers[idx++] = baro_temperature_registers->get_register(1);
        }
    );
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* DataProcessor::get_register_map() const
{
    return register_map;
}

void DataProcessor::update_data_ready_flags_and_write_new_data()
{
    storage::register_fields::register_types_u reg;
    data_ready_register->read(reg.byte, false);

    // Baro data:
    bool could_set_new_baro_data =
        !reg.data_ready.baro_data_ready ||
        baro_pressure_registers->is_read() ||
        baro_temperature_registers->is_read();

    if (could_set_new_baro_data)
    {
        reg.data_ready.baro_data_ready = 0;
        if (has_new_baro_data)
        {
            baro_pressure_registers->write(baro_pressure_data.u8bit, false, false);
            
            baro_temperature_registers->write(baro_temperature_data.u8bit, false, false);

            reg.data_ready.baro_data_ready = 1;
        }
    }

    // Inertial data:
    bool could_set_new_inertial_data =
        !reg.data_ready.gyro_data_ready ||
        !reg.data_ready.accel_data_ready ||
        inertial_gyro_axis_registers->is_read() ||
        inertial_accel_axis_registers->is_read() ||
        inertial_temperature_registers->is_read();

    if (could_set_new_inertial_data)
    {
        reg.data_ready.gyro_data_ready = 0;
        reg.data_ready.accel_data_ready = 0;
        if (has_new_inertial_data)
        {
            inertial_gyro_axis_registers->write(0,  reinterpret_cast<uint8_t*>(&inertial_gyro_data.array[0]), 4, false, false);
            inertial_gyro_axis_registers->write(4,  reinterpret_cast<uint8_t*>(&inertial_gyro_data.array[1]), 4, false, false);
            inertial_gyro_axis_registers->write(8,  reinterpret_cast<uint8_t*>(&inertial_gyro_data.array[2]), 4, false, false);
            inertial_gyro_axis_registers->write(12, reinterpret_cast<uint8_t*>(&inertial_gyro_data.array[3]), 4, false, false);

            inertial_accel_axis_registers->write(0,  reinterpret_cast<uint8_t*>(&inertial_accel_data.array[0]), 4, false, false);
            inertial_accel_axis_registers->write(4,  reinterpret_cast<uint8_t*>(&inertial_accel_data.array[1]), 4, false, false);
            inertial_accel_axis_registers->write(8,  reinterpret_cast<uint8_t*>(&inertial_accel_data.array[2]), 4, false, false);
            inertial_accel_axis_registers->write(12, reinterpret_cast<uint8_t*>(&inertial_accel_data.array[3]), 4, false, false);

            inertial_temperature_registers->write(inertial_temperature_data.u8bit, false, false);

            reg.data_ready.gyro_data_ready = 1;
            reg.data_ready.accel_data_ready = 1;
        }
    }

    // Mag data:
    bool could_set_new_mag_data = 
        !reg.data_ready.mag_data_ready ||
        mag_axis_registers->is_read() ||
        mag_temperature_registers->is_read();

    if (could_set_new_mag_data)
    {
        reg.data_ready.mag_data_ready = 0;
        if (has_new_mag_data)
        {
            mag_axis_registers->write(0,  reinterpret_cast<uint8_t*>(&mag_data.array[0]), 4, false, false);
            mag_axis_registers->write(4,  reinterpret_cast<uint8_t*>(&mag_data.array[1]), 4, false, false);
            mag_axis_registers->write(8,  reinterpret_cast<uint8_t*>(&mag_data.array[2]), 4, false, false);
            mag_axis_registers->write(12, reinterpret_cast<uint8_t*>(&mag_data.array[3]), 4, false, false);

            mag_temperature_registers->write(mag_temperature_data.u8bit, false, false);

            reg.data_ready.mag_data_ready = 1;
        }
    }

    data_ready_register->write(reg.byte, false, false);
}

void DataProcessor::update_register_map()
{
    storage::register_fields::register_types_u reg;

    // Handle register reads.
    update_data_ready_flags_and_write_new_data();

    // Handle register writes.
    uint8_t addr, written_bit_mask, new_value;
    while(register_map->get_next_written_and_changed_register(addr, written_bit_mask, new_value))
	{
        // debug_print("register changed: addr=%02X, written_bit_mask=%02X, new_value=%02X.\n", addr, written_bit_mask, new_value);
        reg.byte = new_value;

        switch(addr)
        {
        case 2:
            if (!has_sensor_error()) board->get_led_pin()->set(reg.led.state);
            break;
        }
        // TODO: react to received data/change.
    }

    if (has_sensor_error()) board->get_led_pin()->set(true);
}

void DataProcessor::update_fusion()
{
    float deltaTime = static_cast<float>(counter->reset_counter0())/1000.0f;
    FusionAhrsUpdate(&ahrs,
        {inertial_gyro_data.axis.x,  inertial_gyro_data.axis.y,  inertial_gyro_data.axis.z},
        {inertial_accel_data.axis.x, inertial_accel_data.axis.y, inertial_accel_data.axis.z},
        {mag_data.axis.x,            mag_data.axis.y,            mag_data.axis.z},
        deltaTime);

    fusion_quaternion = FusionAhrsGetQuaternion(&ahrs);
    quat_registers->write(0,  reinterpret_cast<uint8_t*>(&fusion_quaternion.array[0]), 4, false, false);
    quat_registers->write(4,  reinterpret_cast<uint8_t*>(&fusion_quaternion.array[1]), 4, false, false);
    quat_registers->write(8,  reinterpret_cast<uint8_t*>(&fusion_quaternion.array[2]), 4, false, false);
    quat_registers->write(12, reinterpret_cast<uint8_t*>(&fusion_quaternion.array[3]), 4, false, false);
}

void DataProcessor::update_baro_data(const sensor::pressure_t& pressure, const sensor::temperature_t& temperature)
{
    baro_pressure_data = pressure;
    baro_temperature_data = temperature;
    has_new_baro_data = true;
}

void DataProcessor::update_inertial_data(const sensor::axis3float_t& gyro, const sensor::axis3float_t& accel, const sensor::temperature_t& temperature)
{
    inertial_gyro_data = gyro;
    inertial_accel_data = accel;
    inertial_temperature_data = temperature;
    has_new_inertial_data = true;
    update_fusion();
}

void DataProcessor::update_mag_data(const sensor::axis3float_t& mag, const sensor::temperature_t& temperature)
{
    mag_data = mag;
    mag_temperature_data = temperature;
    has_new_mag_data = true;
    update_fusion();
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
