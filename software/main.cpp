#include <stdio.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"

#include "src/sensor_fusion_board.h"
#include "src/debug_print.h"

#include "src/io/pin/pinio.h"

// #include "src/sensor/lis2mdl-pid/lis2mdl_reg.h"
#include "src/sensor/lps22hb.h"
// #include "src/sensor/lsm6dsm-pid/lsm6dsm_reg.h"

int main()
{
	io::PinIO red_led(LED_PIN_PORT, LED_PIN_MASK);
	io::i2c::I2cMaster i2c_master(io::i2c::i2c_speed_e::I2C_SPEED_400KHZ);

	debug_print("\n\nMAX32660 started with debug info.\n");

	sensor::Lps22hb lps22hb(&i2c_master, 0x5C, BARO_INT_PORT, BARO_INT_MASK, true);


	__enable_irq();

	MXC_Delay(MXC_DELAY_SEC(1));

	lps22hb.begin();

	while (true)
	{
		MXC_Delay(MXC_DELAY_SEC(1));
		red_led.toggle();
		// debug_print("LED TOGGLED.\n");
    }

    return E_NO_ERROR;
}
