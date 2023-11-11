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
    uint32_t u32bit;
    uint8_t u8bit[4]; // only 3 byte actually.
} pressure_t;

typedef union
{
    int16_t i16bit;
    uint8_t u8bit[2];
} temperature_t;

typedef union
{
    uint32_t u32bit;
    uint8_t u8bit[4]; // only 3 byte actually.
} timestamp_t;


} // namespace sensor
