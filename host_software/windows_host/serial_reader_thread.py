import serial
import serial.tools.list_ports
import traceback
import threading
import time
import math
import re
from datetime import datetime
from dataclasses import dataclass

@dataclass
class Euler:
	roll: float = 0.0
	pitch: float = 0.0
	yaw: float = 0.0

@dataclass
class Quat:
	w: float = 1.0
	nx: float = 0.0
	ny:float = 0.0
	nz: float = 0.0

@dataclass
class Axis:
	x: float = 0.0
	y:float = 0.0
	z: float = 0.0

class SerialReaderThread(threading.Thread):
	quat : Quat = Quat()
	euler : Euler = Euler()
	stopped : bool = False

	def __init__(self):
		threading.Thread.__init__(self)
		self.stopped = False
		self.gyro_file = open("gyro.txt", "w")
		self.accel_file = open("accel.txt", "w")
		self.mag_file = open("mag.txt", "w")
		self.start_dt = datetime.now()

	def __del__(self):
		self.gyro_file.close()
		self.accel_file.close()
		self.mag_file.close()

	def find_port(self):
		while not self.stopped:
			ports = serial.tools.list_ports.comports()
			my_port = ""
			for port, desc, hwid in sorted(ports):
				# printing serial hw id so user can match his/her/it own board.
				print("{}: {} [{}]".format(port, desc, hwid))

				# The magic of auto-reconnecting happens here.
				# Most probably you will use a different host MCU.
				# Just expand these if statements.

				if hwid == "USB VID:PID=239A:80AC SER=C7FD1A01A32A":
					my_port = port

				if hwid == "USB VID:PID=303A:80D4 SER=0740D1DA4D8C":
					my_port = port

			if my_port != "":
				print("Selected port is: %s" % my_port)
				return my_port

			time.sleep(2)
		return None

	def open_port(self, my_port):
		return serial.Serial(port=my_port, baudrate=115200, timeout=3)

	def auditlog_gyro(self, gyro:Axis):
		now_dt = datetime.now()
		ts = (now_dt-self.start_dt).total_seconds()
		self.gyro_file.write(f"{ts};{gyro.x};{gyro.y};{gyro.z}\r\n")

	def auditlog_accel(self, accel:Axis):
		now_dt = datetime.now()
		ts = (now_dt-self.start_dt).total_seconds()
		self.accel_file.write(f"{ts};{accel.x};{accel.y};{accel.z}\r\n")

	def auditlog_mag(self, mag:Axis):
		now_dt = datetime.now()
		ts = (now_dt-self.start_dt).total_seconds()
		self.mag_file.write(f"{ts};{mag.x};{mag.y};{mag.z}\r\n")

	def parse_line(self, line):
		parts = re.findall(r'\s|,|[^,\s]+', line)
		parts = [x for x in parts if x.strip() != "" and x.strip() != ","]

		if re.match('quat', parts[0], re.I):
			self.quat = Quat(float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
		elif re.match('euler', parts[0], re.I):
			self.euler = Euler(float(parts[1]), float(parts[2]), float(parts[3]))
		elif re.match('gyro', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_gyro(gyro)
		elif re.match('accel', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_accel(gyro)
		elif re.match('mag', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_mag(gyro)
		elif re.match('err', parts[0], re.I):
			print(f"sensor error: {parts[1]}")

	def read_line(self, my_serial):
		try:
			line = my_serial.readline().decode('UTF-8').replace('\n', '')
			print(line)
			self.parse_line(line)

		except Exception:
			pass
			# print(traceback.format_exc())

	def run(self):
		while not self.stopped:
			my_serial = None
			try:
				my_port = self.find_port()
				if not my_port and self.stopped:
					return
				my_serial = self.open_port(my_port)
				while my_serial.is_open and not self.stopped:
					self.read_line(my_serial)
			except KeyboardInterrupt:
				self.stopped = True
			except Exception:
				traceback.print_exc()
			finally:
				if my_serial:
					my_serial.close()
