import time
import ulab.numpy as np
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

# FLASH_MAGIC                   = const(0xFEEDBEEF)

BRD_REG_ADDR                  = const(0x00)
CTRL_REG_ADDR                 = const(0x01)
STAT_REG_ADDR                 = const(0x02)
PM_REG_ADDR                   = const(0x03)
DRDY_REG_ADDR                 = const(0x04)

QUAT_DATA_REG_ADDR            = const(0x05)
QUAT_DATA_REG_LEN             = const(4*4)

EULER_DATA_REG_ADDR           = const(QUAT_DATA_REG_ADDR + QUAT_DATA_REG_LEN)
EULER_DATA_REG_LEN            = const(3*4)

EARTH_DATA_REG_ADDR           = const(EULER_DATA_REG_ADDR + EULER_DATA_REG_LEN)
EARTH_DATA_REG_LEN            = const(3*4)

GYRO_DATA_REG_ADDR            = const(EARTH_DATA_REG_ADDR + EARTH_DATA_REG_LEN)
GYRO_DATA_REG_LEN             = const(3*4)

ACCEL_DATA_REG_ADDR           = const(GYRO_DATA_REG_ADDR + GYRO_DATA_REG_LEN)
ACCEL_DATA_REG_LEN            = const(3*4)

MAG_DATA_REG_ADDR             = const(ACCEL_DATA_REG_ADDR + ACCEL_DATA_REG_LEN)
MAG_DATA_REG_LEN              = const(3*4)

PRESSURE_DATA_REG_ADDR        = const(MAG_DATA_REG_ADDR + MAG_DATA_REG_LEN)
PRESSURE_DATA_REG_LEN         = const(3)

TEMP_DATA_REG_ADDR            = const(PRESSURE_DATA_REG_ADDR + PRESSURE_DATA_REG_LEN)
TEMP_DATA_REG_LEN             = const(2)

GYRO_MISALIGNMENT_REG_ADDR    = const(TEMP_DATA_REG_ADDR + TEMP_DATA_REG_LEN)
GYRO_MISALIGNMENT_REG_LEN     = const(9*4)

GYRO_SENSITIVITY_REG_ADDR     = const(GYRO_MISALIGNMENT_REG_ADDR + GYRO_MISALIGNMENT_REG_LEN)
GYRO_SENSITIVITY_REG_LEN      = const(3*4)

GYRO_OFFSET_REG_ADDR          = const(GYRO_SENSITIVITY_REG_ADDR + GYRO_SENSITIVITY_REG_LEN)
GYRO_OFFSET_REG_LEN           = const(3*4)

ACCEL_MISALIGNMENT_REG_ADDR   = const(GYRO_OFFSET_REG_ADDR + GYRO_OFFSET_REG_LEN)
ACCEL_MISALIGNMENT_REG_LEN    = const(9*4)

ACCEL_SENSITIVITY_REG_ADDR    = const(ACCEL_MISALIGNMENT_REG_ADDR + ACCEL_MISALIGNMENT_REG_LEN)
ACCEL_SENSITIVITY_REG_LEN     = const(3*4)

ACCEL_OFFSET_REG_ADDR         = const(ACCEL_SENSITIVITY_REG_ADDR + ACCEL_SENSITIVITY_REG_LEN)
ACCEL_OFFSET_REG_LEN          = const(3*4)

SOFT_IRON_MATRIX_REG_ADDR     = const(ACCEL_OFFSET_REG_ADDR + ACCEL_OFFSET_REG_LEN)
SOFT_IRON_MATRIX_REG_LEN      = const(9*4)

HARD_IRON_OFFSET_REG_ADDR     = const(SOFT_IRON_MATRIX_REG_ADDR + SOFT_IRON_MATRIX_REG_LEN)
HARD_IRON_OFFSET_REG_LEN      = const(3*4)

BRD_REG_LED_MASK              = const(0x01)

CTRL_REG_FUSION_START_MASK    = const(0x01)
CTRL_REG_FUSION_STOP_MASK     = const(0x02)
CTRL_REG_CALIB_START_MASK     = const(0x04)
CTRL_REG_CALIB_STOP_MASK      = const(0x08)
CTRL_REG_CALIB_CANCEL_MASK    = const(0x10)
CTRL_REG_CALIB_RESET_MASK     = const(0x20)
CTRL_REG_CALIB_ACTIVE_MASK    = const(0x40)
CTRL_REG_SW_RESTART_MASK      = const(0x80)

STAT_REG_FUSION_RUNNING_MASK  = const(0x01)
STAT_REG_CALIB_UPLOADING_MASK = const(0x02)
STAT_REG_GYRO_ERROR_MASK      = const(0x10)
STAT_REG_ACCEL_ERROR_MASK     = const(0x20)
STAT_REG_MAG_ERROR_MASK       = const(0x40)
STAT_REG_BARO_ERROR_MASK      = const(0x80)
STAT_REG_ERRORS_MASK          = const(0xF0)

# PM_REG_GYRO_MASK            = const(0x03)
# PM_REG_ACCEL_MASK           = const(0x0C)
# PM_REG_MAG_MASK             = const(0x30)
# PM_REG_BARO_MASK            = const(0xC0)

DRDY_REG_QUAT_MASK            = const(0x01)
DRDY_REG_EULER_MASK           = const(0x02)
DRDY_REG_EARTH_MASK           = const(0x04)
DRDY_REG_GYRO_MASK            = const(0x08)
DRDY_REG_ACCEL_MASK           = const(0x10)
DRDY_REG_MAG_MASK             = const(0x20)
DRDY_REG_BARO_MASK            = const(0x40)
DRDY_REG_TEMP_MASK            = const(0x80)

class InertialCalibrationData():
    def __init__(self):
        self.misalignment = np.zeros((3,3), dtype=np.float)
        self.sensitivity = np.zeros((3), dtype=np.float)
        self.offset = np.zeros((3), dtype=np.float)

    def parse_from_buffer(self, buff:bytearray):
        arr = np.frombuffer(buff, dtype=np.float)
        self.misalignment = np.array(arr[0:9], dtype=np.float).reshape((3,3))
        self.sensitivity = np.array(arr[9:12], dtype=np.float)
        self.offset = np.array(arr[12:15], dtype=np.float)

    def to_bytearray(self):
        arr = self.misalignment.tobytes()
        arr.extend(self.sensitivity.tobytes())
        arr.extend(self.offset.tobytes())
        return arr

    def __str__(self):
        out =  f"misalignment:\n{self.misalignment}\n"
        out += f"sensitivity:\n{self.sensitivity}\n"
        out += f"offset:\n{self.offset}\n"
        return out

class MagCalibrationData():
    def __init__(self):
        self.soft_iron_matrix = np.zeros((3,3), dtype=np.float)
        self.hard_iron_offset = np.zeros((3), dtype=np.float)

    def parse_from_buffer(self, buff:bytearray):
        arr = np.frombuffer(buff, dtype=np.float)
        self.soft_iron_matrix = np.array(arr[0:9], dtype=np.float).reshape((3,3))
        self.hard_iron_offset = np.array(arr[9:12], dtype=np.float)

    def to_bytearray(self):
        arr = self.soft_iron_matrix.tobytes()
        arr.extend(self.hard_iron_offset.tobytes())
        return arr

    def __str__(self):
        out =  f"soft iron matrix:\n{self.soft_iron_matrix}\n"
        out += f"hard iron offset:\n{self.hard_iron_offset}\n"
        return out

class CalibrationData():
    def __init__(self):
        self.gyro = InertialCalibrationData()
        self.accel = InertialCalibrationData()
        self.mag = MagCalibrationData()

    def __str__(self):
        out =  f"gyro:\n{str(self.gyro)}\n"
        out += f"accel:\n{str(self.accel)}\n"
        out += f"mag:\n{str(self.mag)}\n"
        return out

class SensorErrors():
    def __init__(self):
        self.gyro = False
        self.accel = False
        self.mag = False
        self.baro = False

    def parse_from_int(self, status:int):
        if status & STAT_REG_GYRO_ERROR_MASK:
            self.gyro  = True
        else:
            self.gyro  = False

        if status & STAT_REG_ACCEL_ERROR_MASK:
            self.accel  = True
        else:
            self.accel  = False

        if status & STAT_REG_MAG_ERROR_MASK:
            self.mag  = True
        else:
            self.mag  = False

        if status & STAT_REG_BARO_ERROR_MASK:
            self.baro  = True
        else:
            self.baro  = False

    def __str__(self):
        out =  f"gyro error: {str(self.gyro)}\n"
        out += f"accel error: {str(self.accel)}\n"
        out += f"mag error: {str(self.mag)}\n"
        out += f"baro error: {str(self.baro)}\n"
        return out

class Powermode:
    POWER_DOWN       = const(0x00)
    LOW_POWER        = const(0x01)
    NORMAL           = const(0x02)
    HIGH_PERFORMANCE = const(0x03)

    def __init__(self, pm):
        self.value = pm

    def __str__(self):
        if self.value == Powermode.POWER_DOWN:
            return "power down"
        if self.value == Powermode.LOW_POWER:
            return "low power"
        if self.value == Powermode.NORMAL:
            return "normal"
        if self.value == Powermode.HIGH_PERFORMANCE:
            return "high performance"

    def __eq__(self, other):
        return self.value == other.value

class SensorPowermodes:
    def __init__(self):
        self.gyro  = Powermode(Powermode.POWER_DOWN)
        self.accel = Powermode(Powermode.POWER_DOWN)
        self.mag   = Powermode(Powermode.POWER_DOWN)
        self.baro  = Powermode(Powermode.POWER_DOWN)

    def parse_from_int(self, pm:int):
        self.gyro  = Powermode(pm & 0x03)
        self.accel = Powermode((pm >> 2) & 0x03)
        self.mag   = Powermode((pm >> 4) & 0x03)
        self.baro  = Powermode((pm >> 6) & 0x03)

    def _check_and_adjust(self):
        if isinstance(self.gyro, int):
            self.gyro = Powermode(self.gyro)
        if isinstance(self.accel, int):
            self.accel = Powermode(self.accel)
        if isinstance(self.mag, int):
            self.mag = Powermode(self.mag)
        if isinstance(self.baro, int):
            self.baro = Powermode(self.baro)
    
    def to_int(self) -> int:
        self._check_and_adjust()
        out = 0
        out = out | (self.gyro.value & 0x03)
        out = out | ((self.accel.value << 2) & 0x03)
        out = out | ((self.mag.value << 4) & 0x03)
        out = out | ((self.baro.value << 6) & 0x03)
        return out

    def __str__(self):
        out =  f"gyro powermode: {str(self.gyro)}\n"
        out += f"accel powermode: {str(self.accel)}\n"
        out += f"mag powermode: {str(self.mag)}\n"
        out += f"baro powermode: {str(self.baro)}\n"
        return out

class SensorDataReady:
    def __init__(self):
        self.quat = False
        self.euler = False
        self.earth = False
        self.gyro = False
        self.accel = False
        self.mag = False
        self.baro = False
        self.temp = False

    def parse_from_int(self, drdy:int):
        if drdy & DRDY_REG_QUAT_MASK:
            self.quat  = True
        else:
            self.quat  = False

        if drdy & DRDY_REG_EULER_MASK:
            self.euler  = True
        else:
            self.euler  = False

        if drdy & DRDY_REG_EARTH_MASK:
            self.earth  = True
        else:
            self.earth  = False

        if drdy & DRDY_REG_GYRO_MASK:
            self.gyro  = True
        else:
            self.gyro  = False

        if drdy & DRDY_REG_ACCEL_MASK:
            self.accel  = True
        else:
            self.accel  = False

        if drdy & DRDY_REG_MAG_MASK:
            self.mag  = True
        else:
            self.mag  = False

        if drdy & DRDY_REG_BARO_MASK:
            self.baro  = True
        else:
            self.baro  = False

        if drdy & DRDY_REG_TEMP_MASK:
            self.temp  = True
        else:
            self.temp  = False

    def __str__(self):
        out =  f"quat drdy: {str(self.quat)}\n"
        out += f"euler drdy: {str(self.euler)}\n"
        out += f"earth drdy: {str(self.earth)}\n"
        out += f"gyro drdy: {str(self.gyro)}\n"
        out += f"accel drdy: {str(self.accel)}\n"
        out += f"mag drdy: {str(self.mag)}\n"
        out += f"baro drdy: {str(self.baro)}\n"
        out += f"temp drdy: {str(self.temp)}\n"
        return out

class SensorFusion:
    def __init__(self, i2c, i2c_ADDR):
        self.device = I2CDevice(i2c, i2c_ADDR)

    def _read_register(self, reg_addr):
        with self.device:
            buff = bytearray(2)
            buff[0] = reg_addr
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            # print(f"read register: {reg_addr:02X} -> {buff[1]:02X}")
            return buff[1]

    def _write_register(self, reg_addr, reg_value):
        # print(f"write register: {reg_addr:02X} -> {reg_value:02X}")
        with self.device:
            buff = bytearray(2)
            buff[0] = reg_addr
            buff[1] = reg_value
            self.device.write(buff)

    def _wait_until_bit_cleared(self, reg_addr, bit) -> bool:
        cnt = 0
        while True:
            time.sleep(0.01)
            reg_value = self._read_register(reg_addr)
            if ((reg_value & bit) == 0):
                return True
            cnt += 1
            if cnt > 50:
                return False
        return False

    def _set_bit(self, value, bit):
        return value | bit

    def _clear_bit(self, value, bit):
        return value & (~bit)

    def _write_bit_set(self, reg_addr, bit):
        reg_value = self._read_register(reg_addr)
        reg_value = self._set_bit(reg_value, bit)
        self._write_register(reg_addr, reg_value)

    def _write_bit_clear(self, reg_addr, bit):
        reg_value = self._read_register(reg_addr)
        reg_value = self._clear_bit(reg_value, bit)
        self._write_register(reg_addr, reg_value)

    def _dump_arr(self, arr):
        return "".join("0x%02X " % i for i in arr)

    def set_led(self, value):
        if value:
            self._write_bit_set(BRD_REG_ADDR, BRD_REG_LED_MASK)
        else:
            self._write_bit_clear(BRD_REG_ADDR, BRD_REG_LED_MASK)

    def software_reset(self):
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_SW_RESTART_MASK)
        time.sleep(0.5)

    def is_calibration_uploading(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_CALIB_UPLOADING_MASK:
            return True
        return False

    def cancel_calibration_upload(self):
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_CALIB_CANCEL_MASK)
        return self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_CALIB_CANCEL_MASK)

    def get_gyro_calibration_data(self) -> InertialCalibrationData:
        gyro_calibration_data = InertialCalibrationData()
        with self.device:
            buff = bytearray(61)
            buff[0] = GYRO_MISALIGNMENT_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            gyro_calibration_data.parse_from_buffer(buff[1:])
        return gyro_calibration_data

    def get_accel_calibration_data(self) -> InertialCalibrationData:
        accel_calibration_data = InertialCalibrationData()
        with self.device:
            buff = bytearray(61)
            buff[0] = ACCEL_MISALIGNMENT_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            accel_calibration_data.parse_from_buffer(buff[1:])
        return accel_calibration_data

    def get_mag_calibration_data(self) -> MagCalibrationData:
        mag_calibration_data = MagCalibrationData()
        with self.device:
            buff = bytearray(49)
            buff[0] = SOFT_IRON_MATRIX_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            mag_calibration_data.parse_from_buffer(buff[1:])
        return mag_calibration_data

    def get_calibration_data(self) -> CalibrationData:
        calibration_data = CalibrationData()
        calibration_data.gyro = self.get_gyro_calibration_data()
        calibration_data.accel = self.get_accel_calibration_data()
        calibration_data.mag = self.get_mag_calibration_data()
        return calibration_data

    def _set_gyro_calibration_data(self, gyro_calibration_data:InertialCalibrationData):
        '''
        Does not starts and stops (applies) the calibration data.
        User must call start_calibration_upload() before and
        stop_calibration_upload() after calling this method.
        '''
        with self.device:
            buff = bytearray(61)
            buff[0] = GYRO_MISALIGNMENT_REG_ADDR
            buff[1:61] = gyro_calibration_data.to_bytearray()
            self.device.write(buff)

    def _set_accel_calibration_data(self, accel_calibration_data:InertialCalibrationData):
        '''
        Does not starts and stops (applies) the calibration data.
        User must call start_calibration_upload() before and
        stop_calibration_upload() after calling this method.
        '''
        with self.device:
            buff = bytearray(61)
            buff[0] = ACCEL_MISALIGNMENT_REG_ADDR
            buff[1:61] = accel_calibration_data.to_bytearray()
            self.device.write(buff)

    def _set_mag_calibration_data(self, mag_calibration_data:MagCalibrationData):
        '''
        Does not starts and stops (applies) the calibration data.
        User must call start_calibration_upload() before and
        stop_calibration_upload() after calling this method.
        '''
        with self.device:
            buff = bytearray(49)
            buff[0] = SOFT_IRON_MATRIX_REG_ADDR
            buff[1:49] = mag_calibration_data.to_bytearray()
            self.device.write(buff)

    def set_calibration_data(self, calibration_data:CalibrationData) -> bool:
        '''
        Because with each write we have to cache, erase and rewrite a whole flash page
        it is preferred to write all calibration data at once.
        '''
        self.start_calibration_upload()
        self._set_gyro_calibration_data(calibration_data.gyro)
        self._set_accel_calibration_data(calibration_data.accel)
        self._set_mag_calibration_data(calibration_data.mag)
        self.stop_calibration_upload()

    def start_calibration_upload(self) -> bool:
        if self.is_fusion_running():
            return False
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_CALIB_START_MASK)
        return self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_CALIB_START_MASK)

    def stop_calibration_upload(self) -> bool:
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_CALIB_STOP_MASK)
        time.sleep(0.1) # writing flash disables i2c slave interrupts for a short period of time.
        return self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_CALIB_STOP_MASK)

    def reset_calibration_data(self) -> bool:
        success = self.start_calibration_upload()
        if not success:
            return False
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_CALIB_RESET_MASK)
        reset_success = self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_CALIB_RESET_MASK)
        success = self.stop_calibration_upload()
        if not success:
            return False
        return reset_success

    def start_fusion(self) -> bool:
        if self.is_calibration_uploading():
            return False
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_FUSION_START_MASK)
        return self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_FUSION_START_MASK)

    def is_fusion_running(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_FUSION_RUNNING_MASK:
            return True
        return False

    def stop_fusion(self) -> bool:
        self._write_bit_set(CTRL_REG_ADDR, CTRL_REG_FUSION_STOP_MASK)
        return self._wait_until_bit_cleared(CTRL_REG_ADDR, CTRL_REG_FUSION_STOP_MASK)


    def is_gyro_sensor_in_error(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_GYRO_ERROR_MASK:
            return True
        return False

    def is_accel_sensor_in_error(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_ACCEL_ERROR_MASK:
            return True
        return False

    def is_mag_sensor_in_error(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_MAG_ERROR_MASK:
            return True
        return False

    def is_baro_sensor_in_error(self) -> bool:
        reg_value = self._read_register(STAT_REG_ADDR)
        if reg_value & STAT_REG_BARO_ERROR_MASK:
            return True
        return False

    def get_sensor_errors(self) -> SensorErrors:
        sensor_errors = SensorErrors()
        sensor_errors.parse_from_int(self._read_register(STAT_REG_ADDR))        
        return sensor_errors

    def get_sensor_powermodes(self) -> SensorPowermodes:
        sensor_powermodes = SensorPowermodes()
        sensor_powermodes.parse_from_int(self._read_register(PM_REG_ADDR))
        return sensor_powermodes

    def set_sensor_powermodes(self, sensor_powermodes:SensorPowermodes) -> bool:
        status = self._read_register(STAT_REG_ADDR)
        if (status & STAT_REG_FUSION_RUNNING_MASK) or (status & STAT_REG_CALIB_UPLOADING_MASK):
            return False
        with self.device:
            buff = bytearray(2)
            buff[0] = PM_REG_ADDR
            buff[1] = sensor_powermodes.to_int()
            self.device.write(buff)
        return True

    def get_sensor_data_ready(self) -> SensorDataReady:
        sensor_data_ready = SensorDataReady()
        sensor_data_ready.parse_from_int(self._read_register(DRDY_REG_ADDR))
        return sensor_data_ready

    def get_quat_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(17)
            buff[0] = QUAT_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float) # w, x, y, z

    def get_euler_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(13)
            buff[0] = EULER_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float) # roll, pitch, yaw

    def get_earth_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(13)
            buff[0] = EARTH_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float)

    def get_gyro_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(13)
            buff[0] = GYRO_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float)

    def get_accel_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(13)
            buff[0] = ACCEL_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float)

    def get_mag_data(self) -> np.ndarray:
        with self.device:
            buff = bytearray(13)
            buff[0] = MAG_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            return np.frombuffer(buff[1:], dtype=np.float)

    def get_pressure_data_hpa(self) -> float:
        with self.device:
            buff = bytearray(4)
            buff[0] = PRESSURE_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            pressure_int = int.from_bytes(buff[1:], "little")
            return pressure_int / 4096.0

    def get_temperature_data(self) -> float:
        with self.device:
            buff = bytearray(3)
            buff[0] = TEMP_DATA_REG_ADDR
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            temp_int = int.from_bytes(buff[1:], "little")
            return temp_int / 100.0

    def get_all_data(self):
        with self.device:
            buff = bytearray(1+1+16+(5*12)+3+2)
            buff[0] = DRDY_REG_ADDR # 1 byte
            self.device.write_then_readinto(buff, buff, out_end=1, in_start=1)
            drdy = SensorDataReady()
            drdy.parse_from_int(buff[1]) # 1 byte
            retval = {}
            retval["drdy"] = drdy
            if drdy.quat: # 16 bytes
                retval["quat"] = np.frombuffer(buff[2:18], dtype=np.float)
            if drdy.euler: # 12 bytes
                retval["euler"] = np.frombuffer(buff[18:30], dtype=np.float)
            if drdy.earth: # 12 bytes
                retval["earth"] = np.frombuffer(buff[30:42], dtype=np.float)
            if drdy.gyro: # 12 bytes
                retval["gyro"] = np.frombuffer(buff[42:54], dtype=np.float)
            if drdy.accel: # 12 bytes
                retval["accel"] = np.frombuffer(buff[54:66], dtype=np.float)
            if drdy.mag: # 12 bytes
                retval["mag"] = np.frombuffer(buff[66:78], dtype=np.float)
            if drdy.baro: # 3 bytes
                retval["baro"] = int.from_bytes(buff[78:81], "little") / 4096.0
            if drdy.temp: # 2 bytes
                retval["temp"] = int.from_bytes(buff[81:83], "little") / 100.0
            return retval
        return None