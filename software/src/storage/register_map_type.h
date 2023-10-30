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
    uint8_t gyro_error  : 1;
    uint8_t accel_error : 1;
    uint8_t mag_error   : 1;
    uint8_t baro_error  : 1;
    uint8_t not_used    : 4;
} sensor_errors_t;

typedef struct
{
    uint8_t gyro_data_ready  : 1;
    uint8_t accel_data_ready : 1;
    uint8_t mag_data_ready   : 1;
    uint8_t baro_data_ready  : 1;
    uint8_t not_used         : 4;
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

#define SENSOR_FUSION_SENSOR_ERRORS    0x00
#define SENSOR_FUSION_DATA_READY       0x01
#define SENSOR_FUSION_RED_LED          0x02

#define SENSOR_FUSION_BARO_PRESSURE_XL 0x03
#define SENSOR_FUSION_BARO_PRESSURE_L  0x04
#define SENSOR_FUSION_BARO_PRESSURE_H  0x05

#define SENSOR_FUSION_BARO_TEMP_L      0x06
#define SENSOR_FUSION_BARO_TEMP_H      0x07

typedef struct __attribute__((packed))
{
    register_types::sensor_errors_t sensor_errors;
    register_types::data_ready_t data_ready;
    register_types::red_led_t red_led;
    uint8_t baro_pressure[3];
    int16_t baro_temp;
} register_map_t;

} // namespace storage
