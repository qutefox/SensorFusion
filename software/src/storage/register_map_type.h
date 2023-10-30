#pragma once

#include <stdint.h>

namespace storage
{
namespace register_types
{

typedef struct 
{
    uint8_t bit0 : 1;
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} bitwise_t;

typedef struct
{
    uint8_t gyro_error : 1;
    uint8_t xl_error   : 1;
    uint8_t mag_error  : 1;
    uint8_t baro_error : 1;
    uint8_t not_used   : 4;
} sensor_errors_t;

typedef struct
{
    uint8_t gyro_data_ready : 1;
    uint8_t xl_data_ready   : 1;
    uint8_t mag_data_ready  : 1;
    uint8_t baro_data_ready : 1;
    uint8_t not_used        : 4;
} data_ready_t;

typedef struct
{
    uint8_t state    : 1;
    uint8_t not_used : 7;
} red_led_t;

typedef union
{
    sensor_errors_t sensor_errors;
    data_ready_t    data_ready;
    red_led_t       red_led;
    bitwise_t       bitwise;
    uint8_t         byte;
} register_types_u;


} // namespace register_types

typedef struct __attribute__((packed))
{
    register_types::sensor_errors_t sensor_errors;
    register_types::data_ready_t data_ready;
    register_types::red_led_t red_led;
    uint8_t baro_pressure[3];
    int16_t baro_temp;
} register_map_t;

} // namespace storage
