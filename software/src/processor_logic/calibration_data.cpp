#include "calibration_data.h"

#include <cstring>

#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/storage/flash.h"
#include "src/storage/register_fields.h"
#include "src/storage/register_map.h"
#include "src/debug_print.h"

#define FLASH_MAGIC_VALUE 0xFEEDBEEF

CalibrationData* CalibrationData::instance = nullptr;
uint32_t CalibrationData::lock = 0;

CalibrationData::CalibrationData()
    : flash{ storage::Flash::get_instance() }
    , register_map{ storage::RegisterMap::get_instance() }
    , register_map_helper{ storage::RegisterMapHelper::get_instance(register_map) }
{
    if (!is_magic_good())
    {
        reset_register_map_calibration_data();
        save_from_register_map_to_flash();
        return;
    }
    read_from_flash_to_register_map();
}

CalibrationData::~CalibrationData()
{

}

CalibrationData* CalibrationData::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new CalibrationData();
    }
    MXC_FreeLock(&lock);
    return instance;
}

bool CalibrationData::is_magic_good()
{
    uint32_t magic_value = 0;
    uint8_t* magic_buffer = reinterpret_cast<uint8_t*>(&magic_value);
    flash->read(0, magic_buffer, 4);
    return magic_value == FLASH_MAGIC_VALUE;
}

void CalibrationData::reset_register_map_calibration_data()
{
    storage::registers::float_u one_f;
    one_f.f = 1.0f;

    storage::registers::float_u zero_f;
    zero_f.f = 0.0f;

    storage::MultiRegisterInterface<uint8_t, uint8_t>* reg;
    reg = register_map->get_gyroscope_misalignment_register();
    // = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    reg->write(0,  one_f.bytes,  4, false); // 1.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f
    reg->write(12, zero_f.bytes, 4, false); // 0.0f
    reg->write(16, one_f.bytes,  4, false); // 1.0f
    reg->write(20, zero_f.bytes, 4, false); // 0.0f
    reg->write(24, zero_f.bytes, 4, false); // 0.0f
    reg->write(28, zero_f.bytes, 4, false); // 0.0f
    reg->write(32, one_f.bytes,  4, false); // 1.0f
    
    reg = register_map->get_gyroscope_sensitivity_register();
    // = {1.0f, 1.0f, 1.0f};
    reg->write(0,  one_f.bytes, 4, false); // 1.0f
    reg->write(4,  one_f.bytes, 4, false); // 1.0f
    reg->write(8,  one_f.bytes, 4, false); // 1.0f

    reg = register_map->get_gyroscope_offset_register();
    // = {0.0f, 0.0f, 0.0f};
    reg->write(0,  zero_f.bytes, 4, false); // 0.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f

    reg = register_map->get_accelerometer_misalignment_register();
    // = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    reg->write(0,  one_f.bytes,  4, false); // 1.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f
    reg->write(12, zero_f.bytes, 4, false); // 0.0f
    reg->write(16, one_f.bytes,  4, false); // 1.0f
    reg->write(20, zero_f.bytes, 4, false); // 0.0f
    reg->write(24, zero_f.bytes, 4, false); // 0.0f
    reg->write(28, zero_f.bytes, 4, false); // 0.0f
    reg->write(32, one_f.bytes,  4, false); // 1.0f

    reg = register_map->get_accelerometer_sensitivity_register();
    // = {1.0f, 1.0f, 1.0f};
    reg->write(0,  one_f.bytes, 4, false); // 1.0f
    reg->write(4,  one_f.bytes, 4, false); // 1.0f
    reg->write(8,  one_f.bytes, 4, false); // 1.0f

    reg = register_map->get_accelerometer_offset_register();
    // = {0.0f, 0.0f, 0.0f};
    reg->write(0,  zero_f.bytes, 4, false); // 0.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f

    reg = register_map->get_soft_iron_matrix_register();
    // = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    reg->write(0,  one_f.bytes,  4, false); // 1.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f
    reg->write(12, zero_f.bytes, 4, false); // 0.0f
    reg->write(16, one_f.bytes,  4, false); // 1.0f
    reg->write(20, zero_f.bytes, 4, false); // 0.0f
    reg->write(24, zero_f.bytes, 4, false); // 0.0f
    reg->write(28, zero_f.bytes, 4, false); // 0.0f
    reg->write(32, one_f.bytes,  4, false); // 1.0f

    reg = register_map->get_hard_iron_offset_register();
    // = {0.0f, 0.0f, 0.0f};
    reg->write(0,  zero_f.bytes, 4, false); // 0.0f
    reg->write(4,  zero_f.bytes, 4, false); // 0.0f
    reg->write(8,  zero_f.bytes, 4, false); // 0.0f
}

void CalibrationData::read_from_flash_to_register_map()
{
    uint8_t buffer[36];

    // Each register map item knows it's own lentgh.
    // Because of that the write() methods does not require length parameter.

    flash->read(4, buffer, 36); // 4 = 0+4
    register_map->get_gyroscope_misalignment_register()->write(buffer, false, false);

    flash->read(40, buffer, 12); // 40 = 4+36
    register_map->get_gyroscope_sensitivity_register()->write(buffer, false, false);

    flash->read(52, buffer, 12); // 52 = 40+12
    register_map->get_gyroscope_offset_register()->write(buffer, false, false);

    flash->read(64, buffer, 36); // 64 = 52+12
    register_map->get_accelerometer_misalignment_register()->write(buffer, false, false);

    flash->read(100, buffer, 12); // 100 = 64+36
    register_map->get_accelerometer_sensitivity_register()->write(buffer, false, false);

    flash->read(112, buffer, 12); // 112 = 100+12
    register_map->get_accelerometer_offset_register()->write(buffer, false, false);

    flash->read(124, buffer, 36); // 124 = 112+12
    register_map->get_soft_iron_matrix_register()->write(buffer, false, false);

    flash->read(160, buffer, 12); // 160 = 124+36
    register_map->get_hard_iron_offset_register()->write(buffer, false, false);
}

void CalibrationData::save_from_register_map_to_flash()
{
    uint8_t buffer[172];

    uint32_t magic_value = FLASH_MAGIC_VALUE;
    uint8_t* magic_buffer = reinterpret_cast<uint8_t*>(&magic_value);
    memcpy(buffer, magic_buffer, 4);

    register_map->get_gyroscope_misalignment_register()->read(buffer+4, false); // 4 = 0+4
    register_map->get_gyroscope_sensitivity_register()->read(buffer+40, false); // 40 = 4+36
    register_map->get_gyroscope_offset_register()->read(buffer+52, false); // 52 = 40+12
    register_map->get_accelerometer_misalignment_register()->read(buffer+64, false); // 64 = 52+12
    register_map->get_accelerometer_sensitivity_register()->read(buffer+100, false); // 100 = 64+36
    register_map->get_accelerometer_offset_register()->read(buffer+112, false); // 112 = 100+12
    register_map->get_soft_iron_matrix_register()->read(buffer+124, false); // 124 = 112+12
    register_map->get_hard_iron_offset_register()->read(buffer+160, false); // 160 = 124+36
    // 172 = 160+12

    flash->write(0, buffer, 172);
}

void CalibrationData::dump_flash_content()
{
    uint8_t buffer[172];
    flash->read(0, buffer, 172);

    // for (uint8_t i = 0 ; i < 172 ; ++i)
    // {
    //     debug_print("0x%02X -> 0x%02X.\n", i , buffer[i]);
    // }

    FusionMatrix matrix;
    FusionVector vector;

    matrix = get_gyroscope_misalignment();
    debug_print("gyroscope misalignment:\n");
    debug_print("%f, %f, %f\n", matrix.element.xx, matrix.element.xy, matrix.element.xz);
    debug_print("%f, %f, %f\n", matrix.element.yx, matrix.element.yy, matrix.element.yz);
    debug_print("%f, %f, %f\n", matrix.element.zx, matrix.element.zy, matrix.element.zz);

    vector = get_gyroscope_sensitivity();
    debug_print("gyroscope sensitivity:\n");
    debug_print("%f, %f, %f\n", vector.axis.x, vector.axis.y, vector.axis.z);

    vector = get_gyroscope_offset();
    debug_print("gyroscope offset:\n");
    debug_print("%f, %f, %f\n", vector.axis.x, vector.axis.y, vector.axis.z);

    matrix = get_accelerometer_misalignment();
    debug_print("accelerometer misalignment:\n");
    debug_print("%f, %f, %f\n", matrix.element.xx, matrix.element.xy, matrix.element.xz);
    debug_print("%f, %f, %f\n", matrix.element.yx, matrix.element.yy, matrix.element.yz);
    debug_print("%f, %f, %f\n", matrix.element.zx, matrix.element.zy, matrix.element.zz);

    vector = get_accelerometer_sensitivity();
    debug_print("accelerometer sensitivity:\n");
    debug_print("%f, %f, %f\n", vector.axis.x, vector.axis.y, vector.axis.z);

    vector = get_accelerometer_offset();
    debug_print("accelerometer offset:\n");
    debug_print("%f, %f, %f\n", vector.axis.x, vector.axis.y, vector.axis.z);

    matrix = get_soft_iron_matrix();
    debug_print("soft iron matrix:\n");
    debug_print("%f, %f, %f\n", matrix.element.xx, matrix.element.xy, matrix.element.xz);
    debug_print("%f, %f, %f\n", matrix.element.yx, matrix.element.yy, matrix.element.yz);
    debug_print("%f, %f, %f\n", matrix.element.zx, matrix.element.zy, matrix.element.zz);

    vector = get_hard_iron_offset();
    debug_print("hard iron offset:\n");
    debug_print("%f, %f, %f\n", vector.axis.x, vector.axis.y, vector.axis.z);
}

FusionMatrix CalibrationData::get_gyroscope_misalignment()
{
    float matrix[9];
    uint8_t* matrix_buffer = reinterpret_cast<uint8_t*>(&matrix);
    register_map->get_gyroscope_misalignment_register()->read(matrix_buffer, false);
    return FusionMatrix{
        matrix[0], matrix[1], matrix[2],
        matrix[3], matrix[4], matrix[5],
        matrix[6], matrix[7], matrix[8]};
}

FusionVector CalibrationData::get_gyroscope_sensitivity()
{
    float vector[3];
    uint8_t* vector_buffer = reinterpret_cast<uint8_t*>(&vector);
    register_map->get_gyroscope_sensitivity_register()->read(vector_buffer, false);
    return FusionVector{vector[0], vector[1], vector[2]};
}

FusionVector CalibrationData::get_gyroscope_offset()
{
    float vector[3];
    uint8_t* vector_buffer = reinterpret_cast<uint8_t*>(&vector);
    register_map->get_gyroscope_offset_register()->read(vector_buffer, false);
    return FusionVector{vector[0], vector[1], vector[2]};
}

FusionMatrix CalibrationData::get_accelerometer_misalignment()
{
    float matrix[9];
    uint8_t* matrix_buffer = reinterpret_cast<uint8_t*>(&matrix);
    register_map->get_accelerometer_misalignment_register()->read(matrix_buffer, false);
    return FusionMatrix{
        matrix[0], matrix[1], matrix[2],
        matrix[3], matrix[4], matrix[5],
        matrix[6], matrix[7], matrix[8]};
}

FusionVector CalibrationData::get_accelerometer_sensitivity()
{
    float vector[3];
    uint8_t* vector_buffer = reinterpret_cast<uint8_t*>(&vector);
    register_map->get_accelerometer_sensitivity_register()->read(vector_buffer, false);
    return FusionVector{vector[0], vector[1], vector[2]};
}

FusionVector CalibrationData::get_accelerometer_offset()
{
    float vector[3];
    uint8_t* vector_buffer = reinterpret_cast<uint8_t*>(&vector);
    register_map->get_accelerometer_offset_register()->read(vector_buffer, false);
    return FusionVector{vector[0], vector[1], vector[2]};
}

FusionMatrix CalibrationData::get_soft_iron_matrix()
{
    float matrix[9];
    uint8_t* matrix_buffer = reinterpret_cast<uint8_t*>(&matrix);
    register_map->get_soft_iron_matrix_register()->read(matrix_buffer, false);
    return FusionMatrix{
        matrix[0], matrix[1], matrix[2],
        matrix[3], matrix[4], matrix[5],
        matrix[6], matrix[7], matrix[8]};
}

FusionVector CalibrationData::get_hard_iron_offset()
{
    float vector[3];
    uint8_t* vector_buffer = reinterpret_cast<uint8_t*>(&vector);
    register_map->get_hard_iron_offset_register()->read(vector_buffer, false);
    return FusionVector{vector[0], vector[1], vector[2]};
}
