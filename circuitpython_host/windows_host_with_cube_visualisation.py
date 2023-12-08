
import subprocess, sys

def pipInstall(package):
	subprocess.call([sys.executable, "-m", "pip", "install", package])

try:
	import pygame
	from pygame.locals import *
except ImportError:
	pipInstall("pygame")
finally:
	import pygame
	from pygame.locals import *

try:
	from OpenGL.GL import *
	from OpenGL.GLU import *
except ImportError:
	pipInstall("pyopengl")
finally:
	from OpenGL.GL import *
	from OpenGL.GLU import *

import time
from datetime import datetime
import math
import re
from dataclasses import dataclass

import serial
import serial.tools.list_ports
import traceback
import threading

vertices= (
	(1, -1, -1),
	(1, 1, -1),
	(-1, 1, -1),
	(-1, -1, -1),
	(1, -1, 1),
	(1, 1, 1),
	(-1, -1, 1),
	(-1, 1, 1)
)

edges = (
	(0,1),
	(0,3),
	(0,4),
	(2,1),
	(2,3),
	(2,7),
	(6,3),
	(6,4),
	(6,7),
	(5,1),
	(5,4),
	(5,7)
)

surfaces = (
	(0,1,2,3),
	(3,2,7,6),
	(6,7,5,4),
	(4,5,1,0),
	(1,5,7,2),
	(4,0,3,6)
)

colors = (
	(255/255,0/255 ,0/255),  # red
	(255/255,99/255,71/255), # tomato
	(178/255,34/255,34/255), # firebrick
	(220/255,20/255,60/255), # crimson

	(0/255  ,128/255,0/255), # green
	(50/255 ,205/255,50/255), # lime green
	(0/255  ,255/255,127/255), # spring green
	(124/255,252/255,0/255), # lawn green

	(0/255  , 0/255 ,255/255), # blue
	(30/255 ,144/255,255/255), # dodger blue
	(100/255,149/255,237/255), # corn flower blue
	(65/255 ,105/255,225/255), # royal blue

	(255/255,165/255,0/255), # orange
	(255/255,215/255,0/255), # gold
	(240/255,230/255,140/255), # khaki
	(255/255,140/255,0/255), # dark orange

	(139/255,0/255,139/255), # magenta
	(128/255,0/255,128/255), # purple
	(148/255,0/255,211/255), # dark violet
	(255/255,20/255,147/255), # deep pink

	(0/255  ,255/255,255/255), # aqua
	(64/255 ,224/255,208/255), # turquoise
	(127/255,255/255,212/255), # aqua marine
	(0/255  ,206/255,209/255), # dark turquoise
)

@dataclass
class YPR:
	yaw: float = 0.0
	pitch: float = 0.0
	roll: float = 0.0

@dataclass
class Quat:
	w: float = 0.0
	nx: float = 0.0
	ny:float = 0.0
	nz: float = 0.0

	def to_YPR(self) -> YPR:
		ypr = YPR()
		w_sq = w * w
		nx_sq = nx * nx
		ny_sq = ny * ny
		nz_sq = nz * nz
		ypr.yaw   = math.atan2(2.0 * (nx * ny + w  * nz), w_sq + nx_sq - ny_sq - nz_sq)
		ypr.pitch = -math.asin(2.0 * (nx * nz - w  * ny))
		ypr.roll  = math.atan2(2.0 * (w  * nx + ny * nz), w_sq - nx_sq - ny_sq + nz_sq)
		ypr.pitch *= 180.0 / math.pi
		ypr.yaw   *= 180.0 / math.pi
		ypr.yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
		ypr.roll  *= 180.0 / math.pi
		return ypr

@dataclass
class Axis:
	x: float = 0.0
	y:float = 0.0
	z: float = 0.0

class SerialReaderThread(threading.Thread):
	quat : Quat = Quat()
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
				print("{}: {} [{}]".format(port, desc, hwid))
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

	def parse_line(self, line) -> Quat:
		parts = re.findall(r'\s|,|[^,\s]+', line)
		parts = [x for x in parts if x.strip() != "" and x.strip() != ","]

		if re.match('quat', parts[0], re.I):
			quat = Quat(float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
			return quat
		elif re.match('gyro', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_gyro(gyro)
		elif re.match('accel', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_accel(gyro)
		elif re.match('mag', parts[0], re.I):
			gyro = Axis(float(parts[1]), float(parts[2]), float(parts[3]))
			self.auditlog_mag(gyro)
		return None

	def read_line(self, my_serial):
		try:
			line = my_serial.readline().decode('UTF-8').replace('\n', '')

			quat = self.parse_line(line)
			if quat is not None:
				self.quat = quat

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
			except Exception:
				traceback.print_exc()
			finally:
				if my_serial:
					my_serial.close()


def resize_window(width, height):
	if height == 0:
		height = 1

	aspect_ratio = width/height
	field_of_view = 45
	zNear = 0.1 #closer clipping plane
	zFar = 100.0 # further clipping plane

	glViewport(0, 0, width, height)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(field_of_view, 1.0*aspect_ratio, zNear, zFar)
	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()

def ui_init(width, height, title):
	video_flags = OPENGL | DOUBLEBUF
	
	pygame.init()
	screen = pygame.display.set_mode((width, height), video_flags)
	pygame.display.set_caption(title)

	resize_window(width, height)

	glShadeModel(GL_SMOOTH)
	glClearColor(0.0, 0.0, 0.0, 0.0)
	glClearDepth(1.0)
	glEnable(GL_DEPTH_TEST)
	glDepthFunc(GL_LEQUAL)
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def cube1():
	glBegin(GL_LINES)
	glColor3fv((0.1,0.1,0.1))
	for edge in edges:
		for vertex in edge:
			glVertex3iv(vertices[vertex])
	glEnd()

def cube2():
	glBegin(GL_QUADS)
	x = 0
	for surface in surfaces:
		for vertex in surface:
			glColor3fv(colors[x])
			x+=1
			glVertex3fv(vertices[vertex])
	glEnd()

def draw(q: Quat):
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

	if not q:
		print("no data")
		return

	glLoadIdentity()
	glTranslatef(0, 0.0, -7.0) # zoom out

	# ypr = q.to_YPR()
	# print("yaw: %f, pitch: %f, roll: %f" % (ypr.yaw, ypr.pitch , ypr.roll))
	# glRotatef(-ypr.roll, 0.00, 0.00, 1.00)
	# glRotatef(ypr.pitch, 1.00, 0.00, 0.00)
	# glRotatef(ypr.yaw, 0.00, 1.00, 0.00)

	# print("Yaw: %f, Pitch: %f, Roll: %f" %(ypr.yaw, ypr.pitch, ypr.roll))

	degrees = 2 * math.acos(q.w) * 180.00/math.pi
	glRotatef(degrees, -1 * q.nx, q.nz, q.ny)

	cube1()
	cube2()

def main():
	serial_reader_thread = SerialReaderThread()
	serial_reader_thread.start()

	try:
		ui_init(640, 480, "Quat visualization")
		stopped = False
		clock = pygame.time.Clock()
		while not stopped:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					stopped = True

			if not stopped:
				quat = serial_reader_thread.quat
				draw(quat)

				pygame.display.flip()
				clock.tick(70)
	except Exception:
		traceback.print_exc()

	print("Closing down application.")

	serial_reader_thread.stopped = True
	serial_reader_thread.join()

if __name__ == '__main__':
	main()