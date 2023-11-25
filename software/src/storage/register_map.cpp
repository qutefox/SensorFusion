#include "src/storage/register_map.h"

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/register.h"
#include "src/storage/register_with_read_flag.h"
#include "src/storage/register_with_write_flag.h"
#include "src/storage/register_multi.h"

using namespace storage;

Registermap* Registermap::instance = nullptr;
uint32_t Registermap::lock = 0;

Registermap::Registermap()
{
    auto make_registers_with_read_flag =
        [](storage::RegisterInterface<uint8_t, uint8_t>** registers, uint8_t length)
        {
            for (uint8_t i = 0 ; i < length ; ++i)
            {
                registers[i] = new storage::RegisterWithReadFlag<uint8_t, uint8_t>(0x00, 0x00);
            }
        };

    board_control_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0x00, 0x00);

    fusion_control_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0x03, 0x00);
    fusion_status_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);

    sensor_control_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0x00, 0x55);
    sensor_status_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);

    sensor_calibration_control_register = new storage::RegisterWithWriteFlag<uint8_t, uint8_t>(0x00, 0x00);
    sensor_calibration_status_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);

    data_ready_register = new storage::Register<uint8_t, uint8_t>(0x00, 0x00);
    
    gyroscope_fusion_registers = new storage::MultiRegister<uint8_t, uint8_t>(12, true, make_registers_with_read_flag);
    accelerometer_fusion_registers = new storage::MultiRegister<uint8_t, uint8_t>(12, true, make_registers_with_read_flag);
    magnetometer_fusion_registers = new storage::MultiRegister<uint8_t, uint8_t>(12, true, make_registers_with_read_flag);
    quaternion_fusion_registers = new storage::MultiRegister<uint8_t, uint8_t>(16, true, make_registers_with_read_flag);
    pressure_registers = new storage::MultiRegister<uint8_t, uint8_t>(3, true, make_registers_with_read_flag);
    temperature_registers = new storage::MultiRegister<uint8_t, uint8_t>(2, true, make_registers_with_read_flag);

    registers = new storage::MultiRegister<uint8_t, uint8_t>(65, false, // 8 + (3*12) + 16 + 3 + 2 = 65
        [this](storage::RegisterInterface<uint8_t, uint8_t>** regs, uint8_t length)
        {
            uint8_t idx = 0;
            regs[idx++] = board_control_register;

            regs[idx++] = fusion_control_register;
            regs[idx++] = fusion_status_register;

            regs[idx++] = sensor_control_register;
            regs[idx++] = sensor_status_register;

            regs[idx++] = sensor_calibration_control_register;
            regs[idx++] = sensor_calibration_status_register;

            regs[idx++] = data_ready_register;

            regs[idx++] = gyroscope_fusion_registers;
            regs[idx++] = gyroscope_fusion_registers->get_register(1);
            regs[idx++] = gyroscope_fusion_registers->get_register(2);
            regs[idx++] = gyroscope_fusion_registers->get_register(3);
            regs[idx++] = gyroscope_fusion_registers->get_register(4);
            regs[idx++] = gyroscope_fusion_registers->get_register(5);
            regs[idx++] = gyroscope_fusion_registers->get_register(6);
            regs[idx++] = gyroscope_fusion_registers->get_register(7);
            regs[idx++] = gyroscope_fusion_registers->get_register(8);
            regs[idx++] = gyroscope_fusion_registers->get_register(9);
            regs[idx++] = gyroscope_fusion_registers->get_register(10);
            regs[idx++] = gyroscope_fusion_registers->get_register(11);

            regs[idx++] = accelerometer_fusion_registers;
            regs[idx++] = accelerometer_fusion_registers->get_register(1);
            regs[idx++] = accelerometer_fusion_registers->get_register(2);
            regs[idx++] = accelerometer_fusion_registers->get_register(3);
            regs[idx++] = accelerometer_fusion_registers->get_register(4);
            regs[idx++] = accelerometer_fusion_registers->get_register(5);
            regs[idx++] = accelerometer_fusion_registers->get_register(6);
            regs[idx++] = accelerometer_fusion_registers->get_register(7);
            regs[idx++] = accelerometer_fusion_registers->get_register(8);
            regs[idx++] = accelerometer_fusion_registers->get_register(9);
            regs[idx++] = accelerometer_fusion_registers->get_register(10);
            regs[idx++] = accelerometer_fusion_registers->get_register(11);

            regs[idx++] = magnetometer_fusion_registers;
            regs[idx++] = magnetometer_fusion_registers->get_register(1);
            regs[idx++] = magnetometer_fusion_registers->get_register(2);
            regs[idx++] = magnetometer_fusion_registers->get_register(3);
            regs[idx++] = magnetometer_fusion_registers->get_register(4);
            regs[idx++] = magnetometer_fusion_registers->get_register(5);
            regs[idx++] = magnetometer_fusion_registers->get_register(6);
            regs[idx++] = magnetometer_fusion_registers->get_register(7);
            regs[idx++] = magnetometer_fusion_registers->get_register(8);
            regs[idx++] = magnetometer_fusion_registers->get_register(9);
            regs[idx++] = magnetometer_fusion_registers->get_register(10);
            regs[idx++] = magnetometer_fusion_registers->get_register(11);

            regs[idx++] = quaternion_fusion_registers;
            regs[idx++] = quaternion_fusion_registers->get_register(1);
            regs[idx++] = quaternion_fusion_registers->get_register(2);
            regs[idx++] = quaternion_fusion_registers->get_register(3);
            regs[idx++] = quaternion_fusion_registers->get_register(4);
            regs[idx++] = quaternion_fusion_registers->get_register(5);
            regs[idx++] = quaternion_fusion_registers->get_register(6);
            regs[idx++] = quaternion_fusion_registers->get_register(7);
            regs[idx++] = quaternion_fusion_registers->get_register(8);
            regs[idx++] = quaternion_fusion_registers->get_register(9);
            regs[idx++] = quaternion_fusion_registers->get_register(10);
            regs[idx++] = quaternion_fusion_registers->get_register(11);
            regs[idx++] = quaternion_fusion_registers->get_register(12);
            regs[idx++] = quaternion_fusion_registers->get_register(13);
            regs[idx++] = quaternion_fusion_registers->get_register(14);
            regs[idx++] = quaternion_fusion_registers->get_register(15);

            regs[idx++] = pressure_registers;
            regs[idx++] = pressure_registers->get_register(1);
            regs[idx++] = pressure_registers->get_register(2);

            regs[idx++] = temperature_registers;
            regs[idx++] = temperature_registers->get_register(1);
        }
    );
}

Registermap::~Registermap()
{
    delete registers;
}

Registermap* Registermap::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Registermap();
    }
    MXC_FreeLock(&lock);
    return instance;
}

void Registermap::reset()
{
    // TODO: 
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_base() const
{
    return registers;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_board_control_register() const
{
    return board_control_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_fusion_control_register() const
{
    return fusion_control_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_fusion_status_register() const
{
    return fusion_status_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_sensor_control_register() const
{
    return sensor_control_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_sensor_status_register() const
{
    return sensor_status_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_sensor_calibration_control_register() const
{
    return sensor_calibration_control_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_sensor_calibration_status_register() const
{
    return sensor_calibration_status_register;
}

storage::RegisterInterface<uint8_t, uint8_t>* Registermap::get_data_ready_register() const
{
    return data_ready_register;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_gyroscope_fusion_registers() const
{
    return gyroscope_fusion_registers;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_accelerometer_fusion_registers() const
{
    return accelerometer_fusion_registers;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_magnetometer_fusion_registers() const
{
    return magnetometer_fusion_registers;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_quaternion_fusion_registers() const
{
    return quaternion_fusion_registers;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_pressure_registers() const
{
    return pressure_registers;
}

storage::MultiRegisterInterface<uint8_t, uint8_t>* Registermap::get_temperature_registers() const
{
    return temperature_registers;
}

