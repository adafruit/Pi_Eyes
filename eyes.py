#!/usr/bin/python

# Eye renderer using pi3d

# Has a heap of dependencies:
# sudo apt-get install pip python-dev python-imaging
# sudo pip install numpy pi3d svg.path

import pi3d
import random
import time
import math
import copy
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from util import *

rails = 32
posts = rails + 1

# Load SVG file, extract paths & convert to point lists --------------------

# OH! Will probably need to make pathToPoints flip the coord system!

dom        = parse("graphics/eye.svg")
vb         = getViewBox(dom)
pointsList = []
pointsList.append(pathToPoints(getPath(dom, "pupilMin"      ), 32, True, True))
pointsList.append(pathToPoints(getPath(dom, "pupilMax"      ), 32, True, True))
pointsList.append(pathToPoints(getPath(dom, "iris"          ), 32, True, True))
pointsList.append(pathToPoints(getPath(dom, "upperLidClosed"), 33, False, True))
pointsList.append(pathToPoints(getPath(dom, "upperLidOpen"  ), 33, False, True))
pointsList.append(pathToPoints(getPath(dom, "upperLidEdge"  ), 33, False, False))
pointsList.append(pathToPoints(getPath(dom, "lowerLidClosed"), 33, False, False))
pointsList.append(pathToPoints(getPath(dom, "lowerLidOpen"  ), 33, False, False))
pointsList.append(pathToPoints(getPath(dom, "lowerLidEdge"  ), 33, False, False))
pointsList.append(pathToPoints(getPath(dom, "scleraFront"   ),  0, False, False))
pointsList.append(pathToPoints(getPath(dom, "scleraBack"    ),  0, False, False))

# To do: gonna keep these as separate objects (with names), not a list.
# Will need to call the scale function manually on each, but no big deal.


# Set up display and initialise pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

if DISPLAY.width <= (DISPLAY.height * 2):
	eyeRadius   = DISPLAY.width / 5
	eyePosition = DISPLAY.width / 4
else:
	eyeRadius   = DISPLAY.height * 2 / 5
	eyePosition = DISPLAY.height / 2

cam    = pi3d.Camera(is_3d=False, at=(0,0,0), eye=(0,0,-1000))
shader = pi3d.Shader("uv_light")
light  = pi3d.Light(lightpos=(0, -500, -500), lightamb=(0.2, 0.2, 0.2))

# Load texture maps --------------------------------------------------------

irisMap   = pi3d.Texture("graphics/iris.jpg"  , mipmap=False,
              filter=pi3d.GL_LINEAR)
scleraMap = pi3d.Texture("graphics/sclera.png", mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)
lidMap    = pi3d.Texture("graphics/lid.png"   , mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)
uvMap     = pi3d.Texture("graphics/uv.png"    , mipmap=False,
              filter=pi3d.GL_LINEAR, blend=False, m_repeat=True)


# Transform point lists to eye dimensions
for p in pointsList: # Each point list
	scalePoints(p, vb, eyeRadius)

# Generate initial iris meshes; the vertex elements are replaced on
# a per-frame basis in the main loop, this just sets up textures, etc.

platt = pfft(pointsList[2], eyeRadius) # Get iris Z depth
zee = platt[0] * 0.99

platt = pfft(pointsList[9], eyeRadius) # scleraFront
angle1 = platt[1]
platt = pfft(pointsList[10], eyeRadius) # scleraBack
angle2 = platt[1]

rightIris = meshInit(32, 4, True, 0, 0.5/irisMap.iy, False)
rightIris.set_textures([irisMap])
rightIris.set_shader(shader)

leftIris = meshInit(32, 4, True, 0.5, 0.5/irisMap.iy, False)
leftIris.set_textures([irisMap])
leftIris.set_shader(shader)

leftUpperEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
leftUpperEyelid.set_textures([lidMap])
leftUpperEyelid.set_shader(shader)

leftLowerEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
leftLowerEyelid.set_textures([lidMap])
leftLowerEyelid.set_shader(shader)

# Generate scleras for each eye
# They're done independently (object isn't re-used) so each may have
# a different image map (or the same image map can be offset on one so
# the repetition isn't obvious)

aaa = 180 - angle1 - angle2
pts=[]
for i in range(24):
#	ca, sa = pi3d.Utility.from_polar(60 - 120 * i / 23)
	ca, sa = pi3d.Utility.from_polar((90 - angle1) - aaa * i / 23)
	pts.append((ca * eyeRadius, sa * eyeRadius))

leftEye = pi3d.Lathe(path=pts, sides=64)
leftEye.set_textures([scleraMap])
leftEye.set_shader(shader)
rotulate(leftEye, 0)

rightEye = pi3d.Lathe(path=pts, sides=64)
rightEye.set_textures([scleraMap])
rightEye.set_shader(shader)
rotulate(rightEye, 0.5)




# Fetch key presses
mykeys = pi3d.Keyboard()

# Have random destination that's +/- 45 degrees
# Have random time-in-flight
# Have random hold time (until next move)
startX       = -30.0  + random.random() * 60.0
startY       = -30.0  + random.random() * 60.0
destX        = startX
destY        = startY
curX         = startX
curY         = startY
moveDuration =   0.075 + random.random() * 0.1
holdDuration =   0.1  + random.random() * 1.0
startTime    =   0.0
isMoving     = False

rot = 0
frames = 0
beginningTime = time.time()

interp = 0.0

rightEye.positionX(-eyePosition)
rightIris.positionX(-eyePosition)

leftEye.positionX(eyePosition)
leftIris.positionX(eyePosition)
leftUpperEyelid.positionX(eyePosition)
leftUpperEyelid.positionZ(-eyeRadius - 42)
leftLowerEyelid.positionX(eyePosition)
leftLowerEyelid.positionZ(-eyeRadius - 42)

state = 0 # Enblink

# Display scene
while DISPLAY.loop_running():

	now = time.time()
	dt  = now - startTime

	frames = frames + 1
#	if(now > beginningTime):
#		print(frames/(now-beginningTime))

	# Eye-moving-around logic
	if isMoving == True:
		if dt <= moveDuration:
			scale        = (now - startTime) / moveDuration
			# Ease in/out curve: 3*t^2-2*t^3
			scale = 3.0 * scale * scale - 2.0 * scale * scale * scale
			curX         = startX + (destX - startX) * scale
			curY         = startY + (destY - startY) * scale
		else:
			startX       = destX
			startY       = destY
			curX         = destX
			curY         = destY
			holdDuration = 0.1 + random.random() * 1.0
			startTime    = now
			isMoving     = False
	else:
		if dt >= holdDuration:
			destX        = -30.0 + random.random() * 60.0
			destY        = -30.0 + random.random() * 60.0
			moveDuration =   0.075 + random.random() *  0.1
			startTime    = now
			isMoving     = True


	# Interpolate between min and max pupil size
	ir = (math.sin(rot*0.02) + 1.0) * 0.5
	ferple = pointsInterp(pointsList[0], pointsList[1], ir)

	# Generate mesh between interpolated pupil and iris bounds
	foont = pointsMesh(None, ferple, pointsList[2], 4, -zee, True)
	# Assign to both eyes
	leftIris.re_init(pts=foont);
	rightIris.re_init(pts=foont);

	convergence = 2.0

	# Right eye (on screen left)

	rightIris.rotateToX(curY)
	rightIris.rotateToY(curX - convergence)
	rightIris.draw()
	rightEye.rotateToX(curY)
	rightEye.rotateToY(curX - convergence)
	rightEye.draw()

	# Left eye (on screen right)

	leftIris.rotateToX(curY)
	leftIris.rotateToY(curX + convergence)
	leftIris.draw()
	leftEye.rotateToX(curY)
	leftEye.rotateToY(curX + convergence)
	leftEye.draw()

	# Eyelid WIP

	# Interpolate a path between 4 and 3
	# 6,7,8 for lower lid
	if state == 0: # enblink
		f1 = pointsInterp(pointsList[4], pointsList[3], 1.0-interp)
		lf1 = pointsInterp(pointsList[7], pointsList[6], 1.0-interp)
		interp += 0.4
		if interp > 1.0:
			interp = 1.0
	 		state = 1
		f2 = pointsInterp(pointsList[4], pointsList[3], 1.0-interp)
		lf2 = pointsInterp(pointsList[7], pointsList[6], 1.0-interp)
	else: # deblink
		f2 = pointsInterp(pointsList[4], pointsList[3], 1.0-interp)
		lf2 = pointsInterp(pointsList[7], pointsList[6], 1.0-interp)
		interp -= 0.2
		if interp < 0.0:
			interp = 0.0
			state = 0
		f1 = pointsInterp(pointsList[4], pointsList[3], 1.0-interp)
		lf1 = pointsInterp(pointsList[7], pointsList[6], 1.0-interp)

	leftUpperEyelid.re_init(pts=pointsMesh(pointsList[5], f2, f1, 5, 0, False))
	#leftUpperEyelid.draw()

	leftLowerEyelid.re_init(pts=pointsMesh(pointsList[8], lf2, lf1, 5, 0, False))
	#leftLowerEyelid.draw()

	rot += 1



	k = mykeys.read()
	if k==27:
		mykeys.close()
		DISPLAY.stop()
		break

