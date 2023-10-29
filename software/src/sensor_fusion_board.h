#pragma once

#include "mxc_device.h"

#define I2C_SLAVE MXC_I2C1
#define I2C_SLAVE_SPEED 400000 // 400 kbit/s (aka i2c fast speed)

#ifndef I2C_SLAVE_ADDR
#define I2C_SLAVE_ADDR 0x51
#endif

#define I2C_MASTER MXC_I2C0
#define I2C_MASTER_SPEED 400000 // 400 kbit/s (aka i2c fast speed)
#define I2C_MASTER_CLOCK_STRETCHING 0

#define LED_PIN_MASK MXC_GPIO_PIN_12

#define BARO_INT_MASK MXC_GPIO_PIN_4

