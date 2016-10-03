#!/usr/bin/python

# Eye renderer using pi3d

# Has a heap of dependencies:
# sudo apt-get install pip python-dev python-imaging
# sudo pip install numpy pi3d svg.path

import pi3d
import random
import time
import math
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from util import *

# Load SVG file, extract paths & convert to point lists --------------------

dom               = parse("graphics/eye.svg")
vb                = getViewBox(dom)
pupilMinPts       = getPoints(dom, "pupilMin"      , 32, True , True )
pupilMaxPts       = getPoints(dom, "pupilMax"      , 32, True , True )
irisPts           = getPoints(dom, "iris"          , 32, True , True )
upperLidClosedPts = getPoints(dom, "upperLidClosed", 33, False, True )
upperLidOpenPts   = getPoints(dom, "upperLidOpen"  , 33, False, True )
upperLidEdgePts   = getPoints(dom, "upperLidEdge"  , 33, False, False)
lowerLidClosedPts = getPoints(dom, "lowerLidClosed", 33, False, False)
lowerLidOpenPts   = getPoints(dom, "lowerLidOpen"  , 33, False, False)
lowerLidEdgePts   = getPoints(dom, "lowerLidEdge"  , 33, False, False)
scleraFrontPts    = getPoints(dom, "scleraFront"   ,  0, False, False)
scleraBackPts     = getPoints(dom, "scleraBack"    ,  0, False, False)

# Set up display and initialize pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

# eyeRadius is the size, in pixels, at which the eye will be rendered
# onscreen.  eyePosition, also pixels, is the offset (left or right)
# from the center point of the screen to the center of each eye.
# This geometry is explained more in-depth in fbx2.c.
if DISPLAY.width <= (DISPLAY.height * 2):
	eyeRadius   = DISPLAY.width / 5
	eyePosition = DISPLAY.width / 4
else:
	eyeRadius   = DISPLAY.height * 2 / 5
	eyePosition = DISPLAY.height / 2

# A 2D camera is used, mostly to allow for pixel-accurate eye placement,
# but also because perspective isn't really helpful or needed here, and
# also this allows eyelids to be handled somewhat easily as flat planes.
# Line of sight is down Z axis, allowing conventional X/Y cartesion
# coords for 2D positions.
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
# U/V map may be useful for debugging texture placement; not normally used
#uvMap     = pi3d.Texture("graphics/uv.png"    , mipmap=False,
#              filter=pi3d.GL_LINEAR, blend=False, m_repeat=True)

# Initialize static geometry -----------------------------------------------

# Transform point lists to eye dimensions
scalePoints(pupilMinPts      , vb, eyeRadius)
scalePoints(pupilMaxPts      , vb, eyeRadius)
scalePoints(irisPts          , vb, eyeRadius)
scalePoints(upperLidClosedPts, vb, eyeRadius)
scalePoints(upperLidOpenPts  , vb, eyeRadius)
scalePoints(upperLidEdgePts  , vb, eyeRadius)
scalePoints(lowerLidClosedPts, vb, eyeRadius)
scalePoints(lowerLidOpenPts  , vb, eyeRadius)
scalePoints(lowerLidEdgePts  , vb, eyeRadius)
scalePoints(scleraFrontPts   , vb, eyeRadius)
scalePoints(scleraBackPts    , vb, eyeRadius)

# Do thresholds for upper and lower eyelids also
irisRegenThreshold = 0.0
# Get bounding rect of scaled pupilMinPts and pupilMaxPts
# This'll give dimensions in pixels
# Determine scale of 1/4 pixel in 0.0-1.0 interpolation

# Generate initial iris meshes; vertex elements will get replaced on
# a per-frame basis in the main loop, this just sets up textures, etc.
rightIris = meshInit(32, 4, True, 0, 0.5/irisMap.iy, False)
rightIris.set_textures([irisMap])
rightIris.set_shader(shader)
# Left iris map U value is offset by 0.5; effectively a 180 degree
# rotation, so it's less obvious that the same texture is in use on both.
leftIris = meshInit(32, 4, True, 0.5, 0.5/irisMap.iy, False)
leftIris.set_textures([irisMap])
leftIris.set_shader(shader)
irisZ = zangle(irisPts, eyeRadius)[0] * 0.99 # Get iris Z depth, for later

# Eyelid meshes are likewise temporary; texture coordinates are
# assigned here but geometry is dynamically regenerated in main loop.
leftUpperEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
leftUpperEyelid.set_textures([lidMap])
leftUpperEyelid.set_shader(shader)
leftLowerEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
leftLowerEyelid.set_textures([lidMap])
leftLowerEyelid.set_shader(shader)

# Generate scleras for each eye...start with a 2D shape for lathing...
angle1 = zangle(scleraFrontPts, eyeRadius)[1] # Sclera front angle
angle2 = zangle(scleraBackPts , eyeRadius)[1] # " back angle
aRange = 180 - angle1 - angle2
pts    = []
for i in range(24):
	ca, sa = pi3d.Utility.from_polar((90 - angle1) - aRange * i / 23)
	pts.append((ca * eyeRadius, sa * eyeRadius))

# Scleras are generated independently (object isn't re-used) so each may
# have a different image map (or the same image map can be offset on one
# so the repetition isn't obvious).
leftEye = pi3d.Lathe(path=pts, sides=64)
leftEye.set_textures([scleraMap])
leftEye.set_shader(shader)
reAxis(leftEye, 0)
rightEye = pi3d.Lathe(path=pts, sides=64)
rightEye.set_textures([scleraMap])
rightEye.set_shader(shader)
reAxis(rightEye, 0.5) # Image map offset = 180 degree rotation

# --------------------------------------------------------------------------


mykeys = pi3d.Keyboard() # For capturing key presses

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

frameCount = 0

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


	if(frameCount == 0):
		# Interpolate between min and max pupil size
		ir = (math.sin(rot*0.02) + 1.0) * 0.5
		ferple = pointsInterp(pupilMinPts, pupilMaxPts, ir)

		# Generate mesh between interpolated pupil and iris bounds
# TO DO: only re-generate pupil if change in size exceeds some threshold
# (maybe 1/4 pixel, since that's resampling limit?)
		foont = pointsMesh(None, ferple, irisPts, 4, -irisZ, True)
		# Assign to both eyes
		leftIris.re_init(pts=foont)
		rightIris.re_init(pts=foont)

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

# To do: only regenerate eyelid if some threshold exceeded
# (i.e. > ~1/4 pixel)
# Determine pupil min and max sizes in pixel space
# Then figure out change in 'interp' range that corresponds
	# Interpolate a path between 4 and 3
	# 6,7,8 for lower lid
#	if state == 0: # enblink
#		f1 = pointsInterp(upperLidOpenPts, upperLidClosedPts, 1.0-interp)
#		lf1 = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 1.0-interp)
#		interp += 0.4
#		if interp > 1.0:
#			interp = 1.0
#	 		state = 1
#		f2 = pointsInterp(upperLidOpenPts, upperLidClosedPts, 1.0-interp)
#		lf2 = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 1.0-interp)
#	else: # deblink
#		f2 = pointsInterp(upperLidOpenPts, upperLidClosedPts, 1.0-interp)
#		lf2 = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 1.0-interp)
#		interp -= 0.2
#		if interp < 0.0:
#			interp = 0.0
#			state = 0
#		f1 = pointsInterp(upperLidOpenPts, upperLidClosedPts, 1.0-interp)
#		lf1 = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 1.0-interp)
#
#	leftUpperEyelid.re_init(pts=pointsMesh(pointsList[5], f2, f1, 5, 0, False))
	#leftUpperEyelid.draw()

#	leftLowerEyelid.re_init(pts=pointsMesh(pointsList[8], lf2, lf1, 5, 0, False))
	#leftLowerEyelid.draw()
# Right eyelid will be a mirror image, unless there's some good reason
# for asymmetry.

	rot += 1



	k = mykeys.read()
	if k==27:
		mykeys.close()
		DISPLAY.stop()
		break

	frameCount += 1

