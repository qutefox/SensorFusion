#include "src/processor_logic/data_processor.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/timer/continuous_timer.h"
#include "src/timer/oneshot_timer.h"
#include "src/storage/register_map.h"
#include "src/sensor/lsm6dsm.h"
#include "src/debug_print.h"

DataProcessor* DataProcessor::instance = nullptr;
uint32_t DataProcessor::lock = 0;

DataProcessor::DataProcessor()
    : register_map{ storage::Registermap::get_instance() }
    , register_map_helper{ storage::RegisterMapHelper::get_instance(register_map) }
    , board{ SensorFusionBoard::get_instance() }
    , continuous_timer{ timer::ContinuousTimer::get_instance() }
    // , oneshot_timer{ timer::OneshotTimer::get_instance() }
    , has_new_pressure_data{ false }
    , has_new_temperature_data{ false }
    , has_new_gyroscope_data{ false }
    , has_new_accelerometer_data{ false }
    , has_new_magnetometer_data{ false }
    , has_new_quaternion_data{ false }
{   
    FusionAhrsInitialise(&ahrs);
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

bool DataProcessor::can_host_control_led()
{
    bool in_error = has_sensor_error();
    bool timer0_started = continuous_timer->is_started(timer::Timers::TIMER0);
    bool host_can_control_led = false;
    if (in_error && !timer0_started)
    {
        debug_print("starting led blink.\n");
        // 2Hz led blink
        continuous_timer->start_timer(timer::Timers::TIMER0, 2,
            [this](void) -> void
            {
                board->get_led_pin()->toggle();
            });
    }
    else if (!in_error)
    {
        if (timer0_started)
        {
            debug_print("stopping led blink.\n");
            continuous_timer->stop_timer(timer::Timers::TIMER0);

            // Set back user value of LED.
            storage::register_fields::register_types_u reg;
            register_map->get_board_control_register()->read(reg.byte, false);
            update_led(reg.board_control);
        }
        host_can_control_led = true;
    }
    return host_can_control_led;
}

void DataProcessor::update_register_map()
{
    handle_register_writes();
    handle_register_reads();
}

void DataProcessor::handle_register_writes()
{
    storage::register_fields::register_types_u reg;
    bool host_can_control_led = can_host_control_led();

    uint8_t addr, written_bit_mask;
    while(register_map->get_base()->get_next_written_and_changed_register(addr, written_bit_mask, reg.byte))
	{
        // debug_print("register changed: addr=%02X, written_bit_mask=%02X, new_value=%02X.\n", addr, written_bit_mask, new_value);
        if (addr == BOARD_CONTROL_REGISTER_ADDRESS)
        {
            if (host_can_control_led && (written_bit_mask & BOARD_CONTROL_LED_MASK))
            {
                update_led(reg.board_control);
            }
        }
        else if (addr == FUSION_CONTROL_REGISTER_ADDRESS)
        {
            if (written_bit_mask & FUSION_CONTROL_START_STOP_MASK)
            {
                update_fusion_start_stop(reg.fusion_control);
            }
        }
        else if (addr == SENSOR_CONTROL_REGISTER_ADDRESS)
        {
            update_sensor_control(reg.sensor_control, written_bit_mask);
        }
    }
}

void DataProcessor::handle_register_reads()
{
    storage::register_fields::data_ready_t drdy = register_map_helper->get_data_ready_flags();


}

void DataProcessor::update_led(storage::register_fields::board_control_t reg)
{
    board->get_led_pin()->write(reg.led);
}

bool DataProcessor::start_sensor_fusion()
{
    register_map_helper->clear_sensor_errors();
    storage::register_fields::register_types_u reg;
    register_map->get_sensor_control_register()->read(reg.byte, false);
    int err = E_NO_ERROR;
    err |= board->get_barometer_sensor()->set_power_mode(0, static_cast<sensor::PowerMode>(reg.sensor_control.baro_powermode));
    err |= board->get_magnetometer_sensor()->set_power_mode(0, static_cast<sensor::PowerMode>(reg.sensor_control.mag_powermode));
    err |= board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO,
        static_cast<sensor::PowerMode>(reg.sensor_control.gyro_powermode));
    err |= board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL,
        static_cast<sensor::PowerMode>(reg.sensor_control.accel_powermode));
    return (err == E_NO_ERROR);
}

void DataProcessor::stop_sensor_fusion()
{
    board->get_barometer_sensor()->set_power_mode(0, sensor::PowerMode::POWER_DOWN);
    board->get_magnetometer_sensor()->set_power_mode(0, sensor::PowerMode::POWER_DOWN);
    board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO, sensor::PowerMode::POWER_DOWN);
    board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL, sensor::PowerMode::POWER_DOWN);
    register_map_helper->clear_sensor_errors();
}

void DataProcessor::update_fusion_start_stop(storage::register_fields::fusion_control_t reg)
{
    if (reg.start_stop)
    {
        // fusion start request received
        FusionAhrsReset(&ahrs);
        bool running = start_sensor_fusion();
        register_map_helper->set_sensor_fusion_running_status(running);
        if (!running) stop_sensor_fusion();
    }
    else
    {
        // fusion stop request received
        stop_sensor_fusion();
        register_map_helper->set_sensor_fusion_running_status(false);
    }
    register_map_helper->clear_fusion_control_start_stop_bit();
}

void DataProcessor::update_sensor_control(storage::register_fields::sensor_control_t sensor_control_reg, uint8_t written_bit_mask)
{
    storage::register_fields::register_types_u reg;
    reg.sensor_control = sensor_control_reg;

    if (register_map_helper->is_fusion_running())
    {
        // Don't allow changing the powermodes while the sensor fusion is running.
        reg.byte = register_map_helper->undo_register_write_change(reg.byte, written_bit_mask);
        register_map->get_sensor_control_register()->write(reg.byte, false, false);
        return;
    }

    // We do not allow turning off of gyroscope and accelerometer.
    // But we allow turning of barometer and magnetometer.
    if (sensor_control_reg.accel_powermode == sensor::PowerMode::POWER_DOWN)
    {
        sensor_control_reg.accel_powermode = sensor::PowerMode::LOW_POWER;
    }

    if (sensor_control_reg.gyro_powermode == sensor::PowerMode::POWER_DOWN)
    {
        sensor_control_reg.gyro_powermode = sensor::PowerMode::LOW_POWER;
    }
    
    register_map->get_sensor_control_register()->write(reg.byte, false, false);
}

void DataProcessor::update_fusion()
{
    // This method should be called repeatedly each time new gyroscope data is available.
    FusionAhrsUpdate(&ahrs, gyroscope_fvect, accelerometer_fvect, magnetometer_fvect, get_time_delta());
    quaternion_fqvect = FusionAhrsGetQuaternion(&ahrs);
    has_new_quaternion_data = true;
}

void DataProcessor::update_gyroscope_fusion_vector(const FusionVector& gyroscope_data)
{
    gyroscope_fvect = gyroscope_data;
    has_new_gyroscope_data = true;
    update_fusion();
}

void DataProcessor::update_accelerometer_fusion_vector(const FusionVector& accelerometer_data)
{
    accelerometer_fvect = accelerometer_data;
    has_new_accelerometer_data = true;
}

void DataProcessor::update_magnetometer_fusion_vector(const FusionVector& magnetometer_data)
{
    magnetometer_fvect = magnetometer_data;
    has_new_magnetometer_data = true;
}

void DataProcessor::update_pressure(sensor::pressure_t pressure_data)
{
    pressure = pressure_data;
    has_new_pressure_data = true;
}

void DataProcessor::update_temperature(sensor::temperature_t temperature_data)
{
    temperature.i16bit = (temperature.i16bit/2) + (temperature_data.i16bit/2);
    has_new_temperature_data = true;
}

void DataProcessor::update_timestamp(sensor::timestamp_t timestamp_data)
{
    timestamp = timestamp_data;
}

float DataProcessor::get_time_delta() // in seconds
{
    // We multiply by 25 because the timestamp generation (in LSM6DSM) has 25 microsecond resolution set.
    uint32_t diff_microseconds = (timestamp.u32bit - previous_timestamp.u32bit) * 25;
    previous_timestamp = timestamp;
    return static_cast<float>(diff_microseconds) / 1000000; // Convert microsecond to second.
}

void DataProcessor::set_gyroscope_sensor_error(bool error)
{
    register_map_helper->set_gyroscope_sensor_error(error);
}

void DataProcessor::set_accelerometer_sensor_error(bool error)
{
    register_map_helper->set_accelerometer_sensor_error(error);
}

void DataProcessor::set_magnetometer_sensor_error(bool error)
{
    register_map_helper->set_magnetometer_sensor_error(error);
}

void DataProcessor::set_barometer_sensor_error(bool error)
{
    register_map_helper->set_barometer_sensor_error(error);
}

bool DataProcessor::has_sensor_error() const
{
    return register_map_helper->has_sensor_error();
}
