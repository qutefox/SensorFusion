#include <stdio.h>
#include <exception>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "lp.h"
#include "uart.h"

#include "src/sensor_fusion_board.h"
#include "src/data_processor.h"

#include "src/debug_print.h"

// https://analog-devices-msdk.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX32660/

int main()
{

#ifdef ENABLE_DEBUG_PRINT
    MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), CONSOLE_BAUD, MAP_A);
#endif

	debug_print("\n\nMAX32660 started with debug info.\n");

	SensorFusionBoard* board = SensorFusionBoard::get_instance();

	debug_print("About to initaise board.\n");

	if (board->begin() != E_NO_ERROR)
	{
		debug_print("Failed to initaise board.\n");
	}
	else
	{
		debug_print("Board successfully initaised.\n");
	}

	DataProcessor* data_processor = DataProcessor::get_instance();

	__enable_irq();

	debug_print("Entering main loop.\n");

	while (true)
	{
		// Changes are made in an interrupt handler where we cannot process them.
		// So we process them here after waking up for any reason.
		board->prepare_for_sleep();
		MXC_LP_EnterSleepMode();
		// MXC_LP_EnterDeepSleepMode();

		// Handle sensor interrupt(s).
		board->get_magnetometer_sensor()->handle_possible_interrupt();
		board->get_inertial_sensor()->handle_possible_interrupt();
		board->get_barometer_sensor()->handle_possible_interrupt();

		// Handle registermap update(s).
		data_processor->update_register_map();
    }

    return E_NO_ERROR;
}
