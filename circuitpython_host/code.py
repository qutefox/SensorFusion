import time
import board

from sensor_fusion import SensorFusion, Powermode

def scan(i2c):
    while not i2c.try_lock():
        pass
    try:
        print(f"I2C addresses found: {[hex(device_address) for device_address in i2c.scan()]}")

    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c.unlock()

i2c = board.I2C()
# scan(i2c)

device = SensorFusion(i2c, 0x51)

if device.is_fusion_running():
    device.stop_fusion()

if device.is_calibration_uploading():
    device.cancel_calibration_upload()

time.sleep(1)

device.start_fusion()

if device.is_fusion_running():
    device.set_led(True)

# for i in range(3):
while True:
    d = device.get_all_data()
    if "quat" in d:
        print(f"quat: {d['quat'][0]}, {d['quat'][1]}, {d['quat'][2]}, {d['quat'][3]}")
    if "gyro" in d:
        print(f"gyro: {d['gyro'][0]}, {d['gyro'][1]}, {d['gyro'][2]}")
    if "accel" in d:
        print(f"accel: {d['accel'][0]}, {d['accel'][1]}, {d['accel'][2]}")
    if "mag" in d:
        print(f"mag: {d['mag'][0]}, {d['mag'][1]}, {d['mag'][2]}")


    # print(f"powermodes: {device.get_sensor_powermodes()}")
    # print(f"drdy: {device.get_sensor_data_ready()}")
    # print(f"errors: {device.get_sensor_errors()}") 
    # print(f"quat: {device.get_quat_data()}")
    # print(f"euler: {device.get_euler_data()}")
    # print(f"earth: {device.get_earth_data()}")
    # print(f"gyro: {device.get_gyro_data()}")
    # print(f"accel: {device.get_accel_data()}")
    # print(f"mag: {device.get_mag_data()}")
    # print(f"pressure: {device.get_pressure_data_hpa()}")
    # print(f"temperature: {device.get_temperature_data()}")
device.stop_fusion()
print("all done")
time.sleep(1)

device.set_led(False)

while True:
    time.sleep(1)
