#include "controller.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/timer/continuous_timer.h"
#include "src/timer/oneshot_timer.h"
#include "src/storage/register_map.h"
#include "src/sensor/lsm6dsm.h"
#include "fusion_data.h"
#include "src/debug_print.h"

Controller* Controller::instance = nullptr;
uint32_t Controller::lock = 0;

Controller::Controller()
    : register_map{ storage::RegisterMap::get_instance() }
    , register_map_helper{ storage::RegisterMapHelper::get_instance(register_map) }
    , board{ SensorFusionBoard::get_instance() }
    , continuous_timer{ timer::ContinuousTimer::get_instance() }
    , fusion_data{ FusionData::get_instance() }
    , calibration_data{ CalibrationData::get_instance() }
{
    // Enable writing to powermode register.
    register_map_helper->set_fusion_running_status(false);
    // Disable writing to calibration data registers.
    register_map_helper->set_calibration_uploading_status(false);

    // calibration_data->dump_flash_content();
}

Controller::~Controller()
{

}

Controller* Controller::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Controller();
    }
    MXC_FreeLock(&lock);
    return instance;
}

void Controller::handle_wakeup()
{
    handle_register_writes();
    fusion_data->update_register_map();
}

void Controller::handle_register_writes()
{
    storage::registers::registers_u reg;
    bool host_can_control_led = can_host_control_led();

    uint8_t addr, written_bit_mask;
    while(register_map->get_base()->get_next_written_and_changed_register(addr, written_bit_mask, reg.byte))
	{
        debug_print("c: addr=%02X, written_bit_mask=%02X, new_value=%02X.\n", addr, written_bit_mask, reg.byte);
        if (addr == CONTROL_REGISTER_ADDRESS)
        {
            if (host_can_control_led && (written_bit_mask & CONTROL_REGISTER_LED_MASK))
            {
                board->get_led_pin()->write(reg.control.led ? 1 : 0);
            }
            if (written_bit_mask & CONTROL_REGISTER_CALIBRATION_ACTIVE_MASK)
            {
                fusion_data->set_use_calibration_data(reg.control.calibration_active ? true : false);
            }
            update_control(reg.control);
        }
        else if (addr == POWERMODE_REGISTER_ADDRESS)
        {
            update_powermode(reg.powermode);
        }
    }
}

bool Controller::can_host_control_led()
{
    bool in_error = register_map_helper->has_sensor_error();
    bool timer0_started = continuous_timer->is_started(timer::Timers::TIMER0);
    if (in_error)
    {
        if (!timer0_started)
        {
            // 5Hz led blink
            continuous_timer->start_timer(timer::Timers::TIMER0, 5,
                [this](void) -> void
                {
                    board->get_led_pin()->toggle();
                });
        }
    }
    else if (!in_error)
    {
        if (timer0_started)
        {
            continuous_timer->stop_timer(timer::Timers::TIMER0);

            // Set back user value of LED.
            storage::registers::registers_u reg;
            register_map->get_control_register()->read(reg.byte, false);
            board->get_led_pin()->write(reg.control.led ? 1 : 0);
        }
        return true;
    }
    return false;
}

void Controller::update_control(storage::registers::control_t reg)
{
    if (reg.fusion_start)
    {
        // fusion start request received
        bool running = false;
        if (!register_map_helper->is_calibration_uploading())
        {
            running = start_fusion();
        }
        register_map_helper->set_fusion_running_status(running);
        if (!running) stop_fusion();
    }
    else if (reg.fusion_stop)
    {
        // fusion stop request received
        stop_fusion();
    }
    else if (reg.calibration_upload_start)
    {
        // Calibration upload start request received.
        if (!register_map_helper->is_fusion_running())
        {
            register_map_helper->set_calibration_uploading_status(true);
        }
        else
        {
            register_map_helper->set_calibration_uploading_status(false);
        }
    }
    else if (reg.calibration_upload_stop)
    {
        // Calibration upload stop request received.
        calibration_data->save_from_register_map_to_flash();
        register_map_helper->set_calibration_uploading_status(false);
        // calibration_data->dump_flash_content();
    }
    else if (reg.calibration_reset)
    {
        // Calibration reset request received.
        if (register_map_helper->is_calibration_uploading() && !register_map_helper->is_fusion_running())
        {
            // We only allow resetting the calibration data when a previous upload start was sent.
            calibration_data->reset_register_map_calibration_data();
        }
    }
    else if (reg.software_restart)
    {
        NVIC_SystemReset();
    }

    register_map_helper->clear_control_bits();
}

void Controller::update_powermode(storage::registers::powermode_t reg)
{
    // We prevent getting here when the fusion is running by setting the write enable
    // flag to false in register_map_helper->set_fusion_running_status(running);

    bool change_needed = false;

    // We do not allow turning off of gyroscope and accelerometer.
    // But we allow turning off barometer and magnetometer.
    if (reg.accel_powermode == sensor::PowerMode::POWER_DOWN)
    {
        reg.accel_powermode = sensor::PowerMode::LOW_POWER;
        change_needed = true;
    }

    if (reg.gyro_powermode == sensor::PowerMode::POWER_DOWN)
    {
        reg.gyro_powermode = sensor::PowerMode::LOW_POWER;
        change_needed = true;
    }
    
    if (change_needed)
    {
        storage::registers::registers_u new_reg;
        new_reg.powermode = reg;
        register_map->get_powermode_register()->write(new_reg.byte, false, false);
    }
}

bool Controller::start_fusion()
{
    reset_fusion_state();
    storage::registers::registers_u reg;
    register_map->get_powermode_register()->read(reg.byte, false);
    int err = E_NO_ERROR;
    err |= board->get_barometer_sensor()->set_power_mode(0, static_cast<sensor::PowerMode>(reg.powermode.baro_powermode));
    err |= board->get_magnetometer_sensor()->set_power_mode(0, static_cast<sensor::PowerMode>(reg.powermode.mag_powermode));
    err |= board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO,
        static_cast<sensor::PowerMode>(reg.powermode.gyro_powermode));
    err |= board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL,
        static_cast<sensor::PowerMode>(reg.powermode.accel_powermode));
    fusion_data->reset();
    fusion_data->update_calibration_data();
    return (err == E_NO_ERROR);
}

void Controller::stop_fusion()
{
    board->get_barometer_sensor()->set_power_mode(0, sensor::PowerMode::POWER_DOWN);
    board->get_magnetometer_sensor()->set_power_mode(0, sensor::PowerMode::POWER_DOWN);
    board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_GYRO, sensor::PowerMode::POWER_DOWN);
    board->get_inertial_sensor()->set_power_mode(sensor::Lsm6dsmDevice::LSM6DSM_DEVICE_ACCEL, sensor::PowerMode::POWER_DOWN);

    reset_fusion_state();
}

void Controller::reset_fusion_state()
{
    register_map_helper->clear_sensor_errors();
    register_map_helper->clear_data_ready_flags();
    register_map_helper->set_fusion_running_status(false);

    // Required so we can fill data registers the first time we get sensor data.
    register_map_helper->set_data_registers_as_read();
}
