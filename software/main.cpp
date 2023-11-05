#include <stdio.h>
#include <exception>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "lp.h"

#include "src/sensor_fusion_board.h"
#include "src/data_processor.h"

#include "src/debug_print.h"

// https://analog-devices-msdk.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX32660/

int main()
{
	debug_print("\n\nMAX32660 started with debug info.\n");

	SensorFusionBoard* board = SensorFusionBoard::get_instance();
	if (board->begin() != E_NO_ERROR)
	{
		debug_print("Failed to initaise board.\n");
	}

	DataProcessor* data_processor = DataProcessor::get_instance();
	if (data_processor->begin() != E_NO_ERROR)
	{
		debug_print("Failed to initaise data processor.\n");
	}

	__enable_irq();

	debug_print("Entering main loop.\n");

	while (true)
	{
		// Changes are made in an interrupt handler where we cannot process them.
		// So we process them here after waking up for any reason.
		debug_print("Entering sleep!\n");
		board->prep_for_sleep();
		MXC_LP_EnterSleepMode();
		debug_print("Woken up!\n");
		data_processor->update_register_map();
    }

	debug_print("Should never get here.\n");

    return E_NO_ERROR;
}
