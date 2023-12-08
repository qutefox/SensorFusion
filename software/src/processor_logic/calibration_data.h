#pragma once

#include "src/Fusion/Fusion/Fusion.h"

#include "src/storage/register_map_interface.h"
#include "src/storage/register_map_helper.h"
#include "src/storage/flash_interface.h"

class CalibrationData
{
private:
    static CalibrationData* instance;
    static uint32_t lock;

    storage::FlashInterface* flash = nullptr;
    storage::RegisterMapInterface* register_map = nullptr;
    storage::RegisterMapHelper* register_map_helper = nullptr;

    bool is_magic_good();

protected:
    CalibrationData();
    virtual ~CalibrationData();

public:
    CalibrationData(CalibrationData& other) = delete;
    void operator=(const CalibrationData& other) = delete;

    static CalibrationData* get_instance();

    void read_from_flash_to_register_map();
    void save_from_register_map_to_flash();
    void reset_register_map_calibration_data();

    FusionMatrix get_gyroscope_misalignment();
    FusionVector get_gyroscope_sensitivity();
    FusionVector get_gyroscope_offset();
    FusionMatrix get_accelerometer_misalignment();
    FusionVector get_accelerometer_sensitivity();
    FusionVector get_accelerometer_offset();
    FusionMatrix get_soft_iron_matrix();
    FusionVector get_hard_iron_offset();
};
