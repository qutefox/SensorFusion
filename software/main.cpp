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

#include "src/storage/register_map.h"

// #include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/sensor/lps22hb.h"
// #include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"

int main()
{
	debug_print("\n\nMAX32660 started with debug info.\n");

	io::pin::Output red_led(MXC_GPIO0, LED_PIN_MASK);
	
	io::i2c::I2cSlave* i2c_slave = io::i2c::I2cSlave::get_instance();
	sensor::Lps22hb* lps22hb = sensor::Lps22hb::get_instance();

	__enable_irq();

	if(i2c_slave->begin() != E_NO_ERROR)
	{
		debug_print("Failed to initalise i2c slave.\n");
	}
	else
	{
		debug_print("Successfully initalised i2c slave.\n");
	}

	io::pin::Input lps22hb_int_pin(MXC_GPIO0, BARO_INT_MASK);
	lps22hb->begin(0x5C, true, &lps22hb_int_pin);

	// TODO: enable low power mode.

	while (true)
	{
		// MXC_Delay(MXC_DELAY_SEC(1));
		// red_led.toggle();
		// i2c_slave->prepare_for_next_transaction();
		// debug_print("");

		i2c_slave->prepare_for_next_transaction();

		while(!i2c_slave->is_current_transaction_done()) {};
    }

    return E_NO_ERROR;
}
