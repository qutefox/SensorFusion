#include "sensor_base.h"

#include "mxc_delay.h"

#include "src/storage/register_map.h"
#include "src/processor_logic/fusion_data.h"
#include "src/io/digital_input_pin_interface.h"
#include "src/io/i2c_device.h"
#include "src/sensor/lps22hb-pid/lps22hb_reg.h"

using namespace sensor;

SensorBase::SensorBase(uint8_t i2c_address, bool i2c_debug)
    : register_map{ storage::RegisterMap::get_instance() }
    , register_map_helper{ storage::RegisterMapHelper::get_instance(register_map) }
    , fusion_data{ FusionData::get_instance() }
    , i2c_device{ new io::I2cDevice(i2c_address, i2c_debug) }
    , dev_ctx{ new stmdev_ctx_t() }
    , interrupt_active{ false }
    , err{ E_NO_ERROR }
{
    i2c_device->begin();

    dev_ctx->handle = i2c_device;
    dev_ctx->write_reg =
        [](void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::I2cDeviceInterface* i2c_device = static_cast<io::I2cDeviceInterface*>(handle);
            return i2c_device->write_bytes(reg, bufp, len);
        };
    dev_ctx->read_reg =
        [](void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) -> int32_t
        {
            io::I2cDeviceInterface* i2c_device = static_cast<io::I2cDeviceInterface*>(handle);
            return i2c_device->read_bytes(reg, bufp, len);
        };
    dev_ctx->mdelay =
        [](uint32_t millisec) -> void
        {
            MXC_Delay(MXC_DELAY_MSEC(millisec));
        };
}

SensorBase::~SensorBase()
{
    end();
    delete i2c_device;
    delete dev_ctx;
}

int SensorBase::end()
{
    return E_NO_ERROR;
}

void SensorBase::set_interrupt_active()
{
    interrupt_active = true;
}

bool SensorBase::has_interrupt()
{
    return interrupt_active;
}

bool SensorBase::has_error()
{
    return err != E_NO_ERROR;
}
