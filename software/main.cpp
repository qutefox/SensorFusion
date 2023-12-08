#include <stdio.h>
#include <exception>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "lp.h"
#include "uart.h"

#include "src/sensor_fusion_board.h"
#include "src/processor_logic/controller.h"

#include "src/debug_print.h"

// https://analog-devices-msdk.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX32660/

int main()
{

#ifdef ENABLE_DEBUG_PRINT
    MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), CONSOLE_BAUD, MAP_A);
#endif

	debug_print("\n\nMAX32660 started with debug info.\n");

	SensorFusionBoard* board = SensorFusionBoard::get_instance();

	debug_print("About to init board.\n");

	int board_err = board->begin();
	if (board_err != E_NO_ERROR)
	{
		debug_print("Failed to initialize board: %d.\n", board_err);
	}
	else
	{
		debug_print("Board successfully initialized.\n");
	}

	Controller* controller = Controller::get_instance();

	__enable_irq();

	while (true)
	{
		if (board->can_go_sleep())
		{
			board->prepare_for_sleep();
			MXC_LP_EnterSleepMode();
			// MXC_LP_EnterDeepSleepMode();
		}

		// Handle sensor interrupt(s).
		board->handle_sensor_interrupts();

		// Controller update.
		controller->handle_wakeup();
	}

    return E_NO_ERROR;
}
