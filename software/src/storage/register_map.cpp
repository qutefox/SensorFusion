#include "src/storage/register_map.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/register.h"
#include "src/storage/register_with_read_flag.h"
#include "src/storage/register_with_write_flag.h"
#include "src/storage/register_multi.h"
#include "src/storage/register_fields.h"

using namespace storage;

RegisterMap* RegisterMap::instance = nullptr;
uint32_t RegisterMap::lock = 0;

RegisterMap::RegisterMap()
{
    auto make_data_registers =
        [](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            uint8_t write_mask = 0x00;
            uint8_t default_value = 0x00;
            for (uint8_t i = 0 ; i < length ; ++i)
            {
                registers[i] = new storage::RegisterWithReadFlag<uint8_t, uint8_t>(write_mask, default_value);
            }
        };

    auto make_calibration_registers =
        [](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            uint8_t write_mask = 0xFF;
            uint8_t default_value = 0x00;
            for (uint8_t i = 0 ; i < length ; ++i)
            {
                registers[i] = new storage::Register<uint8_t, uint8_t>(write_mask, default_value);
            }
        };

    control1_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(CONTROL1_REGISTER_WRITE_MASK, CONTROL1_REGISTER_DEFAULT_VALUE);
    control2_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(CONTROL2_REGISTER_WRITE_MASK, CONTROL2_REGISTER_DEFAULT_VALUE);
    status_register = new storage::Register<uint8_t, uint8_t>(STATUS_REGISTER_WRITE_MASK, STATUS_REGISTER_DEFAULT_VALUE);
    powermode_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(POWERMODE_REGISTER_WRITE_MASK, POWERMODE_REGISTER_DEFAULT_VALUE);
    data_ready_register = new storage::Register<uint8_t, uint8_t>(DATA_READY_REGISTER_WRITE_MASK, DATA_READY_REGISTER_DEFAULT_VALUE);
    
    quaternion_data_register = new storage::MultiRegister<uint8_t, uint8_t>(QUATERNION_DATA_REGISTER_LENGTH, true, make_data_registers);
    euler_data_register = new storage::MultiRegister<uint8_t, uint8_t>(EULER_DATA_REGISTER_LENGTH, true, make_data_registers);
    earth_data_register = new storage::MultiRegister<uint8_t, uint8_t>(EARTH_DATA_REGISTER_LENGTH, true, make_data_registers);
    gyroscope_data_register = new storage::MultiRegister<uint8_t, uint8_t>(GYROSCOPE_DATA_REGISTER_LENGTH, true, make_data_registers);
    accelerometer_data_register = new storage::MultiRegister<uint8_t, uint8_t>(ACCELEROMETER_DATA_REGISTER_LENGTH, true, make_data_registers);
    magnetometer_data_register = new storage::MultiRegister<uint8_t, uint8_t>(MAGNETOMETER_DATA_REGISTER_LENGTH, true, make_data_registers);
    pressure_data_register = new storage::MultiRegister<uint8_t, uint8_t>(PRESSURE_DATA_REGISTER_LENGTH, true, make_data_registers);
    temperature_data_register = new storage::MultiRegister<uint8_t, uint8_t>(TEMPERATURE_DATA_REGISTER_LENGTH, true, make_data_registers);

    gyroscope_misalignment_register = new storage::MultiRegister<uint8_t, uint8_t>(GYROSCOPE_MISALIGNMENT_REGISTER_LENGTH, true, make_calibration_registers);
    gyroscope_sensitivity_register = new storage::MultiRegister<uint8_t, uint8_t>(GYROSCOPE_SENSITIVITY_REGISTER_LENGTH, true, make_calibration_registers);
    gyroscope_offset_register = new storage::MultiRegister<uint8_t, uint8_t>(GYROSCOPE_OFFSET_REGISTER_LENGTH, true, make_calibration_registers);

    accelerometer_misalignment_register = new storage::MultiRegister<uint8_t, uint8_t>(ACCELEROMETER_MISALIGNMENT_REGISTER_LENGTH, true, make_calibration_registers);
    accelerometer_sensitivity_register = new storage::MultiRegister<uint8_t, uint8_t>(ACCELEROMETER_SENSITIVITY_REGISTER_LENGTH, true, make_calibration_registers);
    accelerometer_offset_register = new storage::MultiRegister<uint8_t, uint8_t>(ACCELEROMETER_OFFSET_REGISTER_LENGTH, true, make_calibration_registers);

    soft_iron_matrix_register = new storage::MultiRegister<uint8_t, uint8_t>(SOFT_IRON_MATRIX_REGISTER_LENGTH, true, make_calibration_registers);
    hard_iron_offset_register = new storage::MultiRegister<uint8_t, uint8_t>(HARD_IRON_OFFSET_REGISTER_LENGTH, true, make_calibration_registers);

    registers = new storage::MultiRegister<uint8_t, uint8_t>(REGISTER_MAP_LENGTH, false,
        [this](storage::RegisterInterface<uint8_t, uint8_t>** regs, uint8_t length)
        {
            uint8_t idx = 0;
            regs[idx++] = control1_register;
            regs[idx++] = control2_register;
            regs[idx++] = status_register;
            regs[idx++] = powermode_register;
            regs[idx++] = data_ready_register;

            regs[idx++] = quaternion_data_register;
            for (uint8_t i = 1 ; i < QUATERNION_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = quaternion_data_register->get_register(i);
            }

            regs[idx++] = euler_data_register;
            for (uint8_t i = 1 ; i < EULER_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = euler_data_register->get_register(i);
            }

            regs[idx++] = earth_data_register;
            for (uint8_t i = 1 ; i < EARTH_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = earth_data_register->get_register(i);
            }

            regs[idx++] = gyroscope_data_register;
            for (uint8_t i = 1 ; i < GYROSCOPE_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = gyroscope_data_register->get_register(i);
            }

            regs[idx++] = accelerometer_data_register;
            for (uint8_t i = 1 ; i < ACCELEROMETER_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = accelerometer_data_register->get_register(i);
            }

            regs[idx++] = magnetometer_data_register;
            for (uint8_t i = 1 ; i < MAGNETOMETER_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = magnetometer_data_register->get_register(i);
            }

            regs[idx++] = pressure_data_register;
            for (uint8_t i = 1 ; i < PRESSURE_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = pressure_data_register->get_register(i);
            }

            regs[idx++] = temperature_data_register;
            for (uint8_t i = 1 ; i < TEMPERATURE_DATA_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = temperature_data_register->get_register(i);
            }

            regs[idx++] = gyroscope_misalignment_register;
            for (uint8_t i = 1 ; i < GYROSCOPE_MISALIGNMENT_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = gyroscope_misalignment_register->get_register(i);
            }

            regs[idx++] = gyroscope_sensitivity_register;
            for (uint8_t i = 1 ; i < GYROSCOPE_SENSITIVITY_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = gyroscope_sensitivity_register->get_register(i);
            }

            regs[idx++] = gyroscope_offset_register;
            for (uint8_t i = 1 ; i < GYROSCOPE_OFFSET_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = gyroscope_offset_register->get_register(i);
            }

            regs[idx++] = accelerometer_misalignment_register;
            for (uint8_t i = 1 ; i < ACCELEROMETER_MISALIGNMENT_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = accelerometer_misalignment_register->get_register(i);
            }

            regs[idx++] = accelerometer_sensitivity_register;
            for (uint8_t i = 1 ; i < ACCELEROMETER_SENSITIVITY_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = accelerometer_sensitivity_register->get_register(i);
            }

            regs[idx++] = accelerometer_offset_register;
            for (uint8_t i = 1 ; i < ACCELEROMETER_OFFSET_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = accelerometer_offset_register->get_register(i);
            }

            regs[idx++] = soft_iron_matrix_register;
            for (uint8_t i = 1 ; i < SOFT_IRON_MATRIX_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = soft_iron_matrix_register->get_register(i);
            }

            regs[idx++] = hard_iron_offset_register;
            for (uint8_t i = 1 ; i < HARD_IRON_OFFSET_REGISTER_LENGTH ; ++i)
            {
                regs[idx++] = hard_iron_offset_register->get_register(i);
            }
        }
    );
}

RegisterMap::~RegisterMap()
{
    delete registers;
}

RegisterMap* RegisterMap::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new RegisterMap();
    }
    MXC_FreeLock(&lock);
    return instance;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_base() const
{
    return registers;
}

storage::RegisterInterface<uint8_t, uint8_t>* RegisterMap::get_control1_register() const
{
    return control1_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* RegisterMap::get_control2_register() const
{
    return control2_register;
}


storage::RegisterInterface<uint8_t, uint8_t>* RegisterMap::get_status_register() const
{
    return status_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* RegisterMap::get_powermode_register() const
{
    return powermode_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* RegisterMap::get_data_ready_register() const
{
    return data_ready_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_quaternion_data_register() const
{
    return quaternion_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_euler_data_register() const
{
    return euler_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_earth_data_register() const
{
    return earth_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_gyroscope_data_register() const
{
    return gyroscope_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_accelerometer_data_register() const
{
    return accelerometer_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_magnetometer_data_register() const
{
    return magnetometer_data_register;
}


storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_pressure_data_register() const
{
    return pressure_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_temperature_data_register() const
{
    return temperature_data_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_gyroscope_misalignment_register() const
{
    return gyroscope_misalignment_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_gyroscope_sensitivity_register() const
{
    return gyroscope_sensitivity_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_gyroscope_offset_register() const
{
    return gyroscope_offset_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_accelerometer_misalignment_register() const
{
    return accelerometer_misalignment_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_accelerometer_sensitivity_register() const
{
    return accelerometer_sensitivity_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_accelerometer_offset_register() const
{
    return accelerometer_offset_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_soft_iron_matrix_register() const
{
    return soft_iron_matrix_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* RegisterMap::get_hard_iron_offset_register() const
{
    return hard_iron_offset_register;
}
