#pragma once

#include <stdint.h>

namespace sensor
{

typedef union
{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union
{
    float array[3];
    struct
    {
        float x;
        float y;
        float z;
    } axis;
} axis3float_t;

typedef union
{
    int32_t i32bit;
    uint8_t u8bit[4];
} pressure_t;

typedef union
{
    int16_t i16bit;
    uint8_t u8bit[2];
} temperature_t;


} // namespace sensor
