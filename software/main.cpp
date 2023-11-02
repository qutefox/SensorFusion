#include <stdio.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"

#include "src/sensor_fusion_board.h"
#include "src/debug_print.h"

#include "src/io/output_pin.h"
#include "src/io/input_pin.h"
#include "src/io/i2c_slave.h"

// #include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/sensor/lps22hb.h"
// #include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"

#include "src/data_processor.h"

int main()
{
	debug_print("\n\nMAX32660 started with debug info.\n");

	io::pin::Output red_led(MXC_GPIO0, LED_PIN_MASK);
	DataProcessor* data_processor = DataProcessor::get_instance();
	data_processor->set_red_led_instance(&red_led);

	io::i2c::I2cSlave* i2c_slave = io::i2c::I2cSlave::get_instance();
	sensor::Lps22hb* lps22hb = sensor::Lps22hb::get_instance();

	__enable_irq();

	if(i2c_slave->begin() != E_NO_ERROR)
	{
		debug_print("Failed to initalise i2c slave.\n");
	}

	io::pin::Input lps22hb_int_pin(MXC_GPIO0, BARO_INT_MASK);
	if(lps22hb->begin(0x5C, true, &lps22hb_int_pin) != E_NO_ERROR)
	{
		debug_print("Failed to initalise lps22hb sensor.\n");
	}

	while (true)
	{
		// Ideal would be if we could go to sleep here and an interrupt would wake us up.

		// Changes are made in an interrupt handler where we cannot process them.
		// So we process them in the data processor.
		data_processor->update_register_map();
    }

    return E_NO_ERROR;
}
