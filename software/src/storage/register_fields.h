#pragma once

#include <stdint.h>

namespace storage
{
namespace register_fields
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

#define BOARD_CONTROL_LED_MASK 0x01

typedef struct
{
    uint8_t led : 1;
    uint8_t     : 7;
} board_control_t;

#define FUSION_CONTROL_START_STOP_MASK 0x01

typedef struct
{
    uint8_t start_stop : 1;
    uint8_t            : 7;
} fusion_control_t;

#define FUSION_STATUS_RUNNING_MASK 0x01

typedef struct
{
    uint8_t running : 1;
    uint8_t         : 7;
} fusion_status_t;

typedef struct
{
    uint8_t gyro_powermode  : 2; // sensor::PowerMode
    uint8_t accel_powermode : 2; // sensor::PowerMode
    uint8_t mag_powermode   : 2; // sensor::PowerMode
    uint8_t baro_powermode  : 2; // sensor::PowerMode
} sensor_control_t;

#define SENSOR_STATUS_GYRO_ERROR_MASK 0x01
#define SENSOR_STATUS_ACCEL_ERROR_MASK 0x02
#define SENSOR_STATUS_MAG_ERROR_MASK 0x04
#define SENSOR_STATUS_BARO_ERROR_MASK 0x08

#define SENSOR_STATUS_ERROR_MASK 0x0F

typedef struct
{
    uint8_t gyro_error  : 1;
    uint8_t accel_error : 1;
    uint8_t mag_error   : 1;
    uint8_t baro_error  : 1;
    uint8_t             : 4;
} sensor_status_t;


#define DATA_READY_GYRO_MASK 0x01
#define DATA_READY_ACCEL_MASK 0x02
#define DATA_READY_MAG_MASK 0x04
#define DATA_READY_BARO_MASK 0x08
#define DATA_READY_QUAT_MASK 0x10

typedef struct
{
    uint8_t gyro_data_ready  : 1;
    uint8_t accel_data_ready : 1;
    uint8_t mag_data_ready   : 1;
    uint8_t baro_data_ready  : 1;
    uint8_t quat_data_ready  : 1;
    uint8_t                  : 3;
} data_ready_t;

typedef union
{
    board_control_t   board_control;
    fusion_control_t  fusion_control;
    fusion_status_t   fusion_status;
    sensor_control_t  sensor_control;
    sensor_status_t   sensor_status;
    data_ready_t      data_ready;
    bitwise_t         bitwise;
    uint8_t           byte;
} register_types_u;

} // namespace register_fields
} // namespace storage
