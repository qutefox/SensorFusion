#pragma once

#include <stdint.h>

#include "src/sensor_fusion_board.h"
#include "src/processor_logic/controller_interface.h"
#include "src/processor_logic/fusion_data_interface.h"
#include "src/processor_logic/calibration_data.h"
#include "src/storage/register_map_interface.h"
#include "src/storage/register_map_helper.h"
#include "src/timer/timer_interface.h"
#include "src/Fusion/Fusion/Fusion.h"

class Controller : public ControllerInterface
{
private:
    static Controller* instance;
    static uint32_t lock;

    storage::RegisterMapInterface* register_map = nullptr;
    storage::RegisterMapHelper* register_map_helper = nullptr;
    SensorFusionBoard* board = nullptr;
    timer::TimerInterface* continuous_timer = nullptr;
    FusionDataInterface* fusion_data = nullptr;
    CalibrationData* calibration_data = nullptr;

    void handle_register_writes();

    bool can_host_control_led();
    bool start_fusion();
    void stop_fusion();
    void reset_fusion_state();

    void update_control(storage::registers::control_t reg);
    void update_powermode(storage::registers::powermode_t reg);

protected:
    Controller();
    virtual ~Controller();

public:
    Controller(Controller& other) = delete;
    void operator=(const Controller& other) = delete;

    static Controller* get_instance();

    virtual void handle_wakeup() override;
};
