#pragma once

#include <stdint.h>

namespace storage
{
namespace registers
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

typedef union
{
    float f;
    uint8_t bytes[4];
} float_u;

#define CONTROL1_REGISTER_ADDRESS        0x00
#define CONTROL1_REGISTER_WRITE_MASK     0x83
#define CONTROL1_REGISTER_DEFAULT_VALUE  0x00
#define CONTROL1_REGISTER_EULER_MASK     0x01
#define CONTROL1_REGISTER_EARTH_MASK     0x02
#define CONTROL1_REGISTER_LED_MASK       0x80

typedef struct
{
    uint8_t euler : 1;
    uint8_t earth : 1;
    uint8_t       : 5;
    uint8_t led   : 1;
} control1_t;

#define CONTROL2_REGISTER_ADDRESS                 0x01
#define CONTROL2_REGISTER_WRITE_MASK              0xFF
#define CONTROL2_REGISTER_DEFAULT_VALUE           0x20
#define CONTROL2_REGISTER_FUSION_START_MASK       0x01
#define CONTROL2_REGISTER_FUSION_STOP_MASK        0x02
#define CONTROL2_REGISTER_CALIBRATION_START_MASK  0x04
#define CONTROL2_REGISTER_CALIBRATION_STOP_MASK   0x08
#define CONTROL2_REGISTER_CALIBRATION_CANCEL_MASK 0x10
#define CONTROL2_REGISTER_CALIBRATION_RESET_MASK  0x20
#define CONTROL2_REGISTER_CALIBRATION_ACTIVE_MASK 0x40
#define CONTROL2_REGISTER_SOFTWARE_RESTART_MASK   0x80

typedef struct
{
    uint8_t fusion_start              : 1;
    uint8_t fusion_stop               : 1;
    uint8_t calibration_upload_start  : 1;
    uint8_t calibration_upload_stop   : 1;
    uint8_t calibration_upload_cancel : 1;
    uint8_t calibration_reset         : 1;
    uint8_t calibration_active        : 1; // When 1 then the fusion algo applies calibration, when 0 ignores calibration.
    uint8_t software_restart          : 1;
} control2_t;

#define STATUS_REGISTER_ADDRESS                    0x02
#define STATUS_REGISTER_WRITE_MASK                 0x00
#define STATUS_REGISTER_DEFAULT_VALUE              0x00
#define STATUS_REGISTER_FUSION_RUNNING_MASK        0x01
#define STATUS_REGISTER_CALIBRATION_UPLOADING_MASK 0x02
#define STATUS_REGISTER_FUSION_ERROR_MASK          0x08
#define STATUS_REGISTER_GYRO_ERROR_MASK            0x10
#define STATUS_REGISTER_ACCEL_ERROR_MASK           0x20
#define STATUS_REGISTER_MAG_ERROR_MASK             0x40
#define STATUS_REGISTER_BARO_ERROR_MASK            0x80
#define STATUS_REGISTER_ERRORS_MASK                0xF8

typedef struct
{
    uint8_t fusion_running        : 1;
    uint8_t calibration_uploading : 1;
    uint8_t                       : 1;
    uint8_t fusion_error          : 1;
    uint8_t gyro_error            : 1;
    uint8_t accel_error           : 1;
    uint8_t mag_error             : 1;
    uint8_t baro_error            : 1;
} status_t;

#define POWERMODE_REGISTER_ADDRESS       0x03
#define POWERMODE_REGISTER_WRITE_MASK    0xFF
#define POWERMODE_REGISTER_DEFAULT_VALUE 0x55
#define POWERMODE_REGISTER_GYRO_MASK     0x03
#define POWERMODE_REGISTER_ACCEL_MASK    0x0C
#define POWERMODE_REGISTER_MAG_MASK      0x30
#define POWERMODE_REGISTER_BARO_MASK     0xC0

typedef struct
{
    uint8_t gyro_powermode  : 2; // sensor::PowerMode
    uint8_t accel_powermode : 2; // sensor::PowerMode
    uint8_t mag_powermode   : 2; // sensor::PowerMode
    uint8_t baro_powermode  : 2; // sensor::PowerMode
} powermode_t;

#define DATA_READY_REGISTER_ADDRESS       0x04
#define DATA_READY_REGISTER_WRITE_MASK    0x00
#define DATA_READY_REGISTER_DEFAULT_VALUE 0x00
#define DATA_READY_REGISTER_QUAT_MASK     0x01
#define DATA_READY_REGISTER_EULER_MASK    0x02
#define DATA_READY_REGISTER_EARTH_MASK    0x04
#define DATA_READY_REGISTER_GYRO_MASK     0x08
#define DATA_READY_REGISTER_ACCEL_MASK    0x10
#define DATA_READY_REGISTER_MAG_MASK      0x20
#define DATA_READY_REGISTER_BARO_MASK     0x40
#define DATA_READY_REGISTER_TEMP_MASK     0x80

typedef struct
{
    uint8_t quat_data_ready   : 1;
    uint8_t euler_data_ready  : 1;
    uint8_t earth_data_ready  : 1;
    uint8_t gyro_data_ready   : 1;
    uint8_t accel_data_ready  : 1;
    uint8_t mag_data_ready    : 1;
    uint8_t baro_data_ready   : 1;
    uint8_t temp_data_ready   : 1;
} data_ready_t;

#define QUATERNION_DATA_REGISTER_ADDRESS             0x05
#define QUATERNION_DATA_REGISTER_LENGTH              4*4 // 4 * sizeof(float)

#define EULER_DATA_REGISTER_ADDRESS                  (QUATERNION_DATA_REGISTER_ADDRESS + QUATERNION_DATA_REGISTER_LENGTH)
#define EULER_DATA_REGISTER_LENGTH                   3*4 // 3 * sizeof(float)

#define EARTH_DATA_REGISTER_ADDRESS                  (EULER_DATA_REGISTER_ADDRESS + EULER_DATA_REGISTER_LENGTH)
#define EARTH_DATA_REGISTER_LENGTH                   3*4 // 3 * sizeof(float)

#define GYROSCOPE_DATA_REGISTER_ADDRESS              (EARTH_DATA_REGISTER_ADDRESS + EARTH_DATA_REGISTER_LENGTH)
#define GYROSCOPE_DATA_REGISTER_LENGTH               3*4 // 3 * sizeof(float)

#define ACCELEROMETER_DATA_REGISTER_ADDRESS          (GYROSCOPE_DATA_REGISTER_ADDRESS + GYROSCOPE_DATA_REGISTER_LENGTH)
#define ACCELEROMETER_DATA_REGISTER_LENGTH           3*4 // 3 * sizeof(float)

#define MAGNETOMETER_DATA_REGISTER_ADDRESS           (ACCELEROMETER_DATA_REGISTER_ADDRESS + ACCELEROMETER_DATA_REGISTER_LENGTH)
#define MAGNETOMETER_DATA_REGISTER_LENGTH            3*4 // 3 * sizeof(float)

#define PRESSURE_DATA_REGISTER_ADDRESS               (MAGNETOMETER_DATA_REGISTER_ADDRESS + MAGNETOMETER_DATA_REGISTER_LENGTH)
#define PRESSURE_DATA_REGISTER_LENGTH                3 // 3 bytes

#define TEMPERATURE_DATA_REGISTER_ADDRESS            (PRESSURE_DATA_REGISTER_ADDRESS + PRESSURE_DATA_REGISTER_LENGTH)
#define TEMPERATURE_DATA_REGISTER_LENGTH             2 // 2 bytes

#define GYROSCOPE_MISALIGNMENT_REGISTER_ADDRESS      (TEMPERATURE_DATA_REGISTER_ADDRESS + TEMPERATURE_DATA_REGISTER_LENGTH)
#define GYROSCOPE_MISALIGNMENT_REGISTER_LENGTH       9*4 // 9 * sizeof(float)

#define GYROSCOPE_SENSITIVITY_REGISTER_ADDRESS       (GYROSCOPE_MISALIGNMENT_REGISTER_ADDRESS + GYROSCOPE_MISALIGNMENT_REGISTER_LENGTH)
#define GYROSCOPE_SENSITIVITY_REGISTER_LENGTH        3*4 // 3 * sizeof(float)

#define GYROSCOPE_OFFSET_REGISTER_ADDRESS            (GYROSCOPE_SENSITIVITY_REGISTER_ADDRESS + GYROSCOPE_SENSITIVITY_REGISTER_LENGTH)
#define GYROSCOPE_OFFSET_REGISTER_LENGTH             3*4 // 3 * sizeof(float)

#define ACCELEROMETER_MISALIGNMENT_REGISTER_ADDRESS  (GYROSCOPE_OFFSET_REGISTER_ADDRESS + GYROSCOPE_OFFSET_REGISTER_LENGTH)
#define ACCELEROMETER_MISALIGNMENT_REGISTER_LENGTH   9*4 // 9 * sizeof(float)

#define ACCELEROMETER_SENSITIVITY_REGISTER_ADDRESS   (ACCELEROMETER_MISALIGNMENT_REGISTER_ADDRESS + ACCELEROMETER_MISALIGNMENT_REGISTER_LENGTH)
#define ACCELEROMETER_SENSITIVITY_REGISTER_LENGTH    3*4 // 3 * sizeof(float)

#define ACCELEROMETER_OFFSET_REGISTER_ADDRESS        (ACCELEROMETER_SENSITIVITY_REGISTER_ADDRESS + ACCELEROMETER_SENSITIVITY_REGISTER_LENGTH)
#define ACCELEROMETER_OFFSET_REGISTER_LENGTH         3*4 // 3 * sizeof(float)

#define SOFT_IRON_MATRIX_REGISTER_ADDRESS            (ACCELEROMETER_OFFSET_REGISTER_ADDRESS + ACCELEROMETER_OFFSET_REGISTER_LENGTH)
#define SOFT_IRON_MATRIX_REGISTER_LENGTH             9*4 // 9 * sizeof(float)

#define HARD_IRON_OFFSET_REGISTER_ADDRESS            (SOFT_IRON_MATRIX_REGISTER_ADDRESS + SOFT_IRON_MATRIX_REGISTER_LENGTH)
#define HARD_IRON_OFFSET_REGISTER_LENGTH             3*4 // 3 * sizeof(float)

#define REGISTER_MAP_LENGTH                          (HARD_IRON_OFFSET_REGISTER_ADDRESS + HARD_IRON_OFFSET_REGISTER_LENGTH)

typedef union
{
    control1_t   control1;
    control2_t   control2;
    status_t     status;
    powermode_t  powermode;
    data_ready_t data_ready;
    bitwise_t    bitwise;
    uint8_t      byte;
} registers_u;

} // namespace registers
} // namespace storage
