#pragma once

#include <stdint.h>

namespace io
{
namespace i2c
{

typedef enum : uint8_t
{
    I2C_SPEED_100KHZ,
    I2C_SPEED_400KHZ
} i2c_speed_e;

} // namespace i2c
} // namespace io
