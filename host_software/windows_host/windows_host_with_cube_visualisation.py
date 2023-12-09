import subprocess, sys

def pipInstall(package):
	subprocess.call([sys.executable, "-m", "pip", "install", package])

try:
	from freetype import *
except ImportError:
	pipInstall("freetype-py")
finally:
	from freetype import *

try:
	import numpy as np
except ImportError:
	pipInstall("numpy")
finally:
	import numpy as np

try:
	import OpenGL.GL as gl
	import OpenGL.GLU as glu
	import OpenGL.GLUT as glut
except ImportError:
	# pip by default fucks this up and installs 32 bit version.
	# However I have 64 bit python installed on my machine.
	# That is why we use pre-downloaded whl.
	# Make sure that the whl matches your python version.
	# For 32 bit python use 32 bit whl.
	pipInstall("PyOpenGL-3.1.6-cp39-cp39-win_amd64.whl")
	pipInstall("PyOpenGL_accelerate-3.1.6-cp39-cp39-win_amd64.whl")
finally:
	import OpenGL.GL as gl
	import OpenGL.GLU as glu
	import OpenGL.GLUT as glut

from serial_reader_thread import SerialReaderThread, Quat

import math

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

class Window():
	def __init__(self, width, height, title):
		self.window = None
		self.base = 0
		self.texid = 0

		glut.glutInit(sys.argv)
		glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGB | glut.GLUT_DEPTH)
		self.window = glut.glutCreateWindow(title)
		glut.glutReshapeWindow(width, height)
		glut.glutDisplayFunc(self.on_display)
		glut.glutReshapeFunc(self.on_reshape)
		glut.glutKeyboardFunc(self.on_keyboard)

		gl.glTexEnvf(gl.GL_TEXTURE_ENV, gl.GL_TEXTURE_ENV_MODE, gl.GL_MODULATE)
		# gl.glEnable(gl.GL_DEPTH_TEST)
		# gl.glEnable(gl.GL_BLEND)
		gl.glEnable(gl.GL_COLOR_MATERIAL)
		gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE)
		gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
		gl.glEnable(gl.GL_TEXTURE_2D)

		gl.glShadeModel(gl.GL_SMOOTH)
		gl.glClearColor(0, 0, 0, 0)
		gl.glClearDepth(1)
		gl.glEnable(gl.GL_DEPTH_TEST)
		gl.glDepthFunc(gl.GL_LEQUAL)
		gl.glHint(gl.GL_PERSPECTIVE_CORRECTION_HINT, gl.GL_NICEST)

		self.make_font("JetBrainsMono-Medium.ttf", 64)

		self.serial = SerialReaderThread()
		self.serial.start()

		glut.glutTimerFunc(10, self.timer_event, 1)

		glut.glutMainLoop()

	def __del__(self):
		if self.window is not None:
			glut.glutDestroyWindow(self.window)
			self.window = None
		self.serial.stopped = True
		self.serial.join()

	def timer_event(self, value):
		glut.glutPostRedisplay()
		glut.glutTimerFunc(10, self.timer_event, 1)

	def make_font(self, filename, size):
		 # Load font  and check it is monotype
		face = Face(filename)
		face.set_char_size( size*64 )
		if not face.is_fixed_width:
			raise 'Font is not monotype'

		 # Determine largest glyph size
		width, height, ascender, descender = 0, 0, 0, 0
		for c in range(32,128):
			face.load_char(chr(c), FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT)
			bitmap	= face.glyph.bitmap
			width	 = max( width, bitmap.width )
			ascender  = max( ascender, face.glyph.bitmap_top )
			descender = max( descender, bitmap.rows-face.glyph.bitmap_top )
		height = ascender+descender

		# Generate texture data
		Z = np.zeros((height*6, width*16), dtype=np.ubyte)
		for j in range(6):
			for i in range(16):
				face.load_char(chr(32+j*16+i), FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT)
				bitmap = face.glyph.bitmap
				x = i*width  + face.glyph.bitmap_left
				y = j*height + ascender - face.glyph.bitmap_top
				Z[y:y+bitmap.rows,x:x+bitmap.width].flat = bitmap.buffer

		# Bound texture
		self.texid = gl.glGenTextures(1)
		gl.glBindTexture(gl.GL_TEXTURE_2D, self.texid)
		gl.glTexParameterf(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
		gl.glTexParameterf(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
		gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_ALPHA, Z.shape[1], Z.shape[0], 0,
						gl.GL_ALPHA, gl.GL_UNSIGNED_BYTE, Z)

		# Generate display lists
		dx, dy = width/float(Z.shape[1]), height/float(Z.shape[0])
		self.base = gl.glGenLists(8*16)
		for i in range(8*16):
			c = chr(i)
			x = i%16
			y = i//16-2
			gl.glNewList(self.base+i, gl.GL_COMPILE)
			if (c == '\n'):
				gl.glPopMatrix()
				gl.glTranslatef( 0, -height, 0)
				gl.glPushMatrix()
			elif (c == '\t'):
				gl.glTranslatef(4*width, 0, 0)
			elif (i >= 32):
				gl.glBegin(gl.GL_QUADS)
				gl.glTexCoord2f((x  )*dx, (y+1)*dy), gl.glVertex(0,	 -height)
				gl.glTexCoord2f((x  )*dx, (y  )*dy), gl.glVertex(0,	 0 )
				gl.glTexCoord2f((x+1)*dx, (y  )*dy), gl.glVertex(width, 0 )
				gl.glTexCoord2f((x+1)*dx, (y+1)*dy), gl.glVertex(width, -height)
				gl.glEnd()
				gl.glTranslatef(width, 0, 0)
			gl.glEndList()

	def draw_text(self, text, x, y):
		gl.glEnable(gl.GL_BLEND)
		gl.glBindTexture(gl.GL_TEXTURE_2D, self.texid)
		gl.glColor(1,1,1,1) # white
		gl.glPushMatrix( )
		gl.glTranslate(x, y, -5)
		gl.glPushMatrix()
		gl.glScalef(0.0035, 0.0035, 0.0035)
		gl.glListBase(self.base)
		gl.glCallLists([ord(c) for c in text])
		gl.glPopMatrix()
		gl.glPopMatrix()
		gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
		gl.glDisable(gl.GL_BLEND)

	def draw_cube_lines(self):
		gl.glBegin(gl.GL_LINES)
		gl.glColor3fv((0.1,0.1,0.1))
		for edge in edges:
			for vertex in edge:
				gl.glVertex3iv(vertices[vertex])
		gl.glEnd()

	def draw_cube_surfaces(self):
		gl.glBegin(gl.GL_QUADS)
		x = 0
		for surface in surfaces:
			for vertex in surface:
				gl.glColor3fv(colors[x])
				x+=1
				gl.glVertex3fv(vertices[vertex])
		gl.glEnd()

	def on_display(self):
		gl.glClearColor(0,0,0,0)
		gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

		gl.glLoadIdentity()
		gl.glTranslatef(0, 0, -7.0) # zoom out
		q = self.serial.quat
		degrees = 2 * math.acos(q.w) * 180.00/math.pi
		gl.glRotatef(degrees, -1 * q.nx, q.nz, q.ny)
		self.draw_cube_lines()
		self.draw_cube_surfaces()

		gl.glLoadIdentity()
		self.draw_text(f"Quat: {q.w:.3f}, {q.nx:.3f}, {q.ny:.3f}, {q.nz:.3f}", -2.5, -1.2)
		euler = self.serial.euler
		self.draw_text(f"Roll: {euler.roll:.1f}, Pitch: {euler.pitch:.1f}, Yaw: {euler.yaw:.1f}", -2.5, -1.5)

		glut.glutSwapBuffers()

	def on_reshape(self, width, height):
		if height == 0:
			height = 1

		aspect_ratio = width/height
		field_of_view = 45
		zNear = 0.1 #closer clipping plane
		zFar = 100.0 # further clipping plane

		gl.glViewport(0, 0, width, height)
		gl.glMatrixMode(gl.GL_PROJECTION)
		gl.glLoadIdentity()
		glu.gluPerspective(field_of_view, 1.0*aspect_ratio, zNear, zFar)
		gl.glMatrixMode(gl.GL_MODELVIEW)
		gl.glLoadIdentity()

	def on_keyboard(self, key, x, y):
		int_key = int.from_bytes(key, "little")
		if int_key == 27: # ESC key
			glut.glutDestroyWindow(self.window)
			self.window = None

if __name__ == '__main__':
	window = Window(640, 480, "Quat visualization")