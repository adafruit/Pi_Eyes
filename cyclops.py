#!/usr/bin/python

# This is a PARED-DOWN version of eyes.py designed for the Gakken
# WorldEye display.  It renders a single eye (centered on screen) and
# does NOT require the OLED or TFT displays...doesn't even require the
# Snake Eyes Bonnet if you just have it running in autonomous mode.
# Code is just as in-progress as eyes.py and could use some work.

import math
import pi3d
import random
import threading
import time
import RPi.GPIO as GPIO
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from gfxutil import *
from snake_eyes_bonnet import SnakeEyesBonnet

# INPUT CONFIG for eye motion ----------------------------------------------
# ANALOG INPUTS REQUIRE SNAKE EYES BONNET

JOYSTICK_X_IN   = -1    # Analog input for eye horiz pos (-1 = auto)
JOYSTICK_Y_IN   = -1    # Analog input for eye vert position (")
PUPIL_IN        = -1    # Analog input for pupil control (-1 = auto)
JOYSTICK_X_FLIP = False # If True, reverse stick X axis
JOYSTICK_Y_FLIP = False # If True, reverse stick Y axis
PUPIL_IN_FLIP   = False # If True, reverse reading from PUPIL_IN
TRACKING        = True  # If True, eyelid tracks pupil
PUPIL_SMOOTH    = 16    # If > 0, filter input from PUPIL_IN
PUPIL_MIN       = 0.0   # Lower analog range from PUPIL_IN
PUPIL_MAX       = 1.0   # Upper "
BLINK_PIN       = 23    # GPIO pin for blink button
AUTOBLINK       = True  # If True, eye blinks autonomously


# GPIO initialization ------------------------------------------------------

GPIO.setmode(GPIO.BCM)
if BLINK_PIN >= 0: GPIO.setup(BLINK_PIN , GPIO.IN, pull_up_down=GPIO.PUD_UP)


# ADC stuff ----------------------------------------------------------------

# ADC channels are read and stored in a separate thread to avoid slowdown
# from blocking operations. The animation loop can read at its leisure.

if JOYSTICK_X_IN >= 0 or JOYSTICK_Y_IN >= 0 or PUPIL_IN >= 0:
	bonnet = SnakeEyesBonnet(daemon=True)
	bonnet.setup_channel(JOYSTICK_X_IN, reverse=JOYSTICK_X_FLIP)
	bonnet.setup_channel(JOYSTICK_Y_IN, reverse=JOYSTICK_Y_FLIP)
	bonnet.setup_channel(PUPIL_IN, reverse=PUPIL_IN_FLIP)
	bonnet.start()


# Load SVG file, extract paths & convert to point lists --------------------

# Thanks Glen Akins for the symmetrical-lidded cyclops eye SVG!
# Iris & pupil have been scaled down slightly in this version to compensate
# for how the WorldEye distorts things...looks OK on WorldEye now but might
# seem small and silly if used with the regular OLED/TFT code.
dom               = parse("graphics/cyclops-eye.svg")
vb                = get_view_box(dom)
pupilMinPts       = get_points(dom, "pupilMin"      , 32, True , True )
pupilMaxPts       = get_points(dom, "pupilMax"      , 32, True , True )
irisPts           = get_points(dom, "iris"          , 32, True , True )
scleraFrontPts    = get_points(dom, "scleraFront"   ,  0, False, False)
scleraBackPts     = get_points(dom, "scleraBack"    ,  0, False, False)
upperLidClosedPts = get_points(dom, "upperLidClosed", 33, False, True )
upperLidOpenPts   = get_points(dom, "upperLidOpen"  , 33, False, True )
upperLidEdgePts   = get_points(dom, "upperLidEdge"  , 33, False, False)
lowerLidClosedPts = get_points(dom, "lowerLidClosed", 33, False, False)
lowerLidOpenPts   = get_points(dom, "lowerLidOpen"  , 33, False, False)
lowerLidEdgePts   = get_points(dom, "lowerLidEdge"  , 33, False, False)


# Set up display and initialize pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

# eyeRadius is the size, in pixels, at which the whole eye will be rendered.
if DISPLAY.width <= (DISPLAY.height * 2):
	# For WorldEye, eye size is -almost- full screen height
	eyeRadius   = DISPLAY.height / 2.1
else:
	eyeRadius   = DISPLAY.height * 2 / 5

# A 2D camera is used, mostly to allow for pixel-accurate eye placement,
# but also because perspective isn't really helpful or needed here, and
# also this allows eyelids to be handled somewhat easily as 2D planes.
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
scale_points(pupilMinPts      , vb, eyeRadius)
scale_points(pupilMaxPts      , vb, eyeRadius)
scale_points(irisPts          , vb, eyeRadius)
scale_points(scleraFrontPts   , vb, eyeRadius)
scale_points(scleraBackPts    , vb, eyeRadius)
scale_points(upperLidClosedPts, vb, eyeRadius)
scale_points(upperLidOpenPts  , vb, eyeRadius)
scale_points(upperLidEdgePts  , vb, eyeRadius)
scale_points(lowerLidClosedPts, vb, eyeRadius)
scale_points(lowerLidOpenPts  , vb, eyeRadius)
scale_points(lowerLidEdgePts  , vb, eyeRadius)

# Regenerating flexible object geometry (such as eyelids during blinks, or
# iris during pupil dilation) is CPU intensive, can noticably slow things
# down, especially on single-core boards.  To reduce this load somewhat,
# determine a size change threshold below which regeneration will not occur;
# roughly equal to 1/2 pixel, since 2x2 area sampling is used.

# Determine change in pupil size to trigger iris geometry regen
irisRegenThreshold = 0.0
a = points_bounds(pupilMinPts) # Bounds of pupil at min size (in pixels)
b = points_bounds(pupilMaxPts) # " at max size
maxDist = max(abs(a[0] - b[0]), abs(a[1] - b[1]), # Determine distance of max
              abs(a[2] - b[2]), abs(a[3] - b[3])) # variance around each edge
# maxDist is motion range in pixels as pupil scales between 0.0 and 1.0.
# 1.0 / maxDist is one pixel's worth of scale range.  Need 1/2 that...
if maxDist > 0: irisRegenThreshold = 0.5 / maxDist

# Determine change in eyelid values needed to trigger geometry regen.
# This is done a little differently than the pupils...instead of bounds,
# the distance between the middle points of the open and closed eyelid
# paths is evaluated, then similar 1/2 pixel threshold is determined.
upperLidRegenThreshold = 0.0
lowerLidRegenThreshold = 0.0
p1 = upperLidOpenPts[len(upperLidOpenPts) // 2]
p2 = upperLidClosedPts[len(upperLidClosedPts) // 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: upperLidRegenThreshold = 0.5 / math.sqrt(d)
p1 = lowerLidOpenPts[len(lowerLidOpenPts) // 2]
p2 = lowerLidClosedPts[len(lowerLidClosedPts) // 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: lowerLidRegenThreshold = 0.5 / math.sqrt(d)

# Generate initial iris mesh; vertex elements will get replaced on
# a per-frame basis in the main loop, this just sets up textures, etc.
iris = mesh_init((32, 4), (0, 0.5 / irisMap.iy), True, False)
iris.set_textures([irisMap])
iris.set_shader(shader)
irisZ = zangle(irisPts, eyeRadius)[0] * 0.99 # Get iris Z depth, for later

# Eyelid meshes are likewise temporary; texture coordinates are
# assigned here but geometry is dynamically regenerated in main loop.
upperEyelid = mesh_init((33, 5), (0, 0.5 / lidMap.iy), False, True)
upperEyelid.set_textures([lidMap])
upperEyelid.set_shader(shader)
lowerEyelid = mesh_init((33, 5), (0, 0.5 / lidMap.iy), False, True)
lowerEyelid.set_textures([lidMap])
lowerEyelid.set_shader(shader)

# Generate sclera for eye...start with a 2D shape for lathing...
angle1 = zangle(scleraFrontPts, eyeRadius)[1] # Sclera front angle
angle2 = zangle(scleraBackPts , eyeRadius)[1] # " back angle
aRange = 180 - angle1 - angle2
pts    = []
for i in range(24):
	ca, sa = pi3d.Utility.from_polar((90 - angle1) - aRange * i / 23)
	pts.append((ca * eyeRadius, sa * eyeRadius))

eye = pi3d.Lathe(path=pts, sides=64)
eye.set_textures([scleraMap])
eye.set_shader(shader)
re_axis(eye, 0.0)


# Init global stuff --------------------------------------------------------

mykeys = pi3d.Keyboard() # For capturing key presses

startX       = random.uniform(-30.0, 30.0)
n            = math.sqrt(900.0 - startX * startX)
startY       = random.uniform(-n, n)
destX        = startX
destY        = startY
curX         = startX
curY         = startY
moveDuration = random.uniform(0.075, 0.175)
holdDuration = random.uniform(0.1, 1.1)
startTime    = 0.0
isMoving     = False

frames        = 0
beginningTime = time.time()

eye.positionX(0.0)
iris.positionX(0.0)
upperEyelid.positionX(0.0)
upperEyelid.positionZ(-eyeRadius - 42)
lowerEyelid.positionX(0.0)
lowerEyelid.positionZ(-eyeRadius - 42)

currentPupilScale  =  0.5
prevPupilScale     = -1.0 # Force regen on first frame
prevUpperLidWeight = 0.5
prevLowerLidWeight = 0.5
prevUpperLidPts    = points_interp(upperLidOpenPts, upperLidClosedPts, 0.5)
prevLowerLidPts    = points_interp(lowerLidOpenPts, lowerLidClosedPts, 0.5)

ruRegen = True
rlRegen = True

timeOfLastBlink = 0.0
timeToNextBlink = 1.0
blinkState      = 0
blinkDuration   = 0.1
blinkStartTime  = 0

trackingPos = 0.3

# Generate one frame of imagery
def frame(p):

	global startX, startY, destX, destY, curX, curY
	global moveDuration, holdDuration, startTime, isMoving
	global frames
	global iris
	global pupilMinPts, pupilMaxPts, irisPts, irisZ
	global eye
	global upperEyelid, lowerEyelid
	global upperLidOpenPts, upperLidClosedPts, lowerLidOpenPts, lowerLidClosedPts
	global upperLidEdgePts, lowerLidEdgePts
	global prevUpperLidPts, prevLowerLidPts
	global prevUpperLidWeight, prevLowerLidWeight
	global prevPupilScale
	global irisRegenThreshold, upperLidRegenThreshold, lowerLidRegenThreshold
	global luRegen, llRegen, ruRegen, rlRegen
	global timeOfLastBlink, timeToNextBlink
	global blinkState
	global blinkDuration
	global blinkStartTime
	global trackingPos

	DISPLAY.loop_running()

	now = time.time()
	dt  = now - startTime

	frames += 1
#	if(now > beginningTime):
#		print(frames/(now-beginningTime))

	if JOYSTICK_X_IN >= 0 and JOYSTICK_Y_IN >= 0:
		# Eye position from analog inputs
		curX = bonnet.channel[JOYSTICK_X_IN].value
		curY = bonnet.channel[JOYSTICK_Y_IN].value
		curX = -30.0 + curX * 60.0
		curY = -30.0 + curY * 60.0
	else :
		# Autonomous eye position
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
				holdDuration = random.uniform(0.15, 1.7)
				startTime    = now
				isMoving     = False
		else:
			if dt >= holdDuration:
				destX        = random.uniform(-30.0, 30.0)
				n            = math.sqrt(900.0 - destX * destX)
				destY        = random.uniform(-n, n)
				# Movement is slower in this version because
				# the WorldEye display is big and the eye
				# should have some 'mass' to it.
				moveDuration = random.uniform(0.12, 0.35)
				startTime    = now
				isMoving     = True


	# Regenerate iris geometry only if size changed by >= 1/2 pixel
	if abs(p - prevPupilScale) >= irisRegenThreshold:
		# Interpolate points between min and max pupil sizes
		interPupil = points_interp(pupilMinPts, pupilMaxPts, p)
		# Generate mesh between interpolated pupil and iris bounds
		mesh = points_mesh((None, interPupil, irisPts), 4, -irisZ, True)
		iris.re_init(pts=mesh)
		prevPupilScale = p

	# Eyelid WIP

	if AUTOBLINK and (now - timeOfLastBlink) >= timeToNextBlink:
		# Similar to movement, eye blinks are slower in this version
		timeOfLastBlink = now
		duration        = random.uniform(0.06, 0.12)
		if blinkState != 1:
			blinkState     = 1 # ENBLINK
			blinkStartTime = now
			blinkDuration  = duration
		timeToNextBlink = duration * 3 + random.uniform(0.0, 4.0)

	if blinkState: # Eye currently winking/blinking?
		# Check if blink time has elapsed...
		if (now - blinkStartTime) >= blinkDuration:
			# Yes...increment blink state, unless...
			if (blinkState == 1 and # Enblinking and...
			    (BLINK_PIN >= 0 and    # blink pin held
			     GPIO.input(BLINK_PIN) == GPIO.LOW)):
				# Don't advance yet; eye is held closed
				pass
			else:
				blinkState += 1
				if blinkState > 2:
					blinkState = 0 # NOBLINK
				else:
					blinkDuration *= 2.0
					blinkStartTime = now
	else:
		if BLINK_PIN >= 0 and GPIO.input(BLINK_PIN) == GPIO.LOW:
			blinkState     = 1 # ENBLINK
			blinkStartTime = now
			blinkDuration  = random.uniform(0.035, 0.06)

	if TRACKING:
		# 0 = fully up, 1 = fully down
		n = 0.5 - curY / 70.0
		if   n < 0.0: n = 0.0
		elif n > 1.0: n = 1.0
		trackingPos = (trackingPos * 3.0 + n) * 0.25

	if blinkState:
		n = (now - blinkStartTime) / blinkDuration
		if n > 1.0: n = 1.0
		if blinkState == 2: n = 1.0 - n
	else:
		n = 0.0
	newUpperLidWeight = trackingPos + (n * (1.0 - trackingPos))
	newLowerLidWeight = (1.0 - trackingPos) + (n * trackingPos)

	if (ruRegen or (abs(newUpperLidWeight - prevUpperLidWeight) >=
	  upperLidRegenThreshold)):
		newUpperLidPts = points_interp(upperLidOpenPts,
		  upperLidClosedPts, newUpperLidWeight)
		if newUpperLidWeight > prevUpperLidWeight:
			upperEyelid.re_init(pts=points_mesh(
			  (upperLidEdgePts, prevUpperLidPts,
			  newUpperLidPts), 5, 0, False))
		else:
			upperEyelid.re_init(pts=points_mesh(
			  (upperLidEdgePts, newUpperLidPts,
			  prevUpperLidPts), 5, 0, False))
		prevUpperLidWeight = newUpperLidWeight
		prevUpperLidPts    = newUpperLidPts
		ruRegen = True
	else:
		ruRegen = False

	if (rlRegen or (abs(newLowerLidWeight - prevLowerLidWeight) >=
	  lowerLidRegenThreshold)):
		newLowerLidPts = points_interp(lowerLidOpenPts,
		  lowerLidClosedPts, newLowerLidWeight)
		if newLowerLidWeight > prevLowerLidWeight:
			lowerEyelid.re_init(pts=points_mesh(
			  (lowerLidEdgePts, prevLowerLidPts,
			  newLowerLidPts), 5, 0, False))
		else:
			lowerEyelid.re_init(pts=points_mesh(
			  (lowerLidEdgePts, newLowerLidPts,
			  prevLowerLidPts), 5, 0, False))
		prevLowerLidWeight = newLowerLidWeight
		prevLowerLidPts    = newLowerLidPts
		rlRegen = True
	else:
		rlRegen = False

	# Draw eye

	iris.rotateToX(curY)
	iris.rotateToY(curX)
	iris.draw()
	eye.rotateToX(curY)
	eye.rotateToY(curX)
	eye.draw()
	upperEyelid.draw()
	lowerEyelid.draw()

	k = mykeys.read()
	if k==27:
		mykeys.close()
		DISPLAY.stop()
		exit(0)


def split( # Recursive simulated pupil response when no analog sensor
  startValue, # Pupil scale starting value (0.0 to 1.0)
  endValue,   # Pupil scale ending value (")
  duration,   # Start-to-end time, floating-point seconds
  range):     # +/- random pupil scale at midpoint
	startTime = time.time()
	if range >= 0.125: # Limit subdvision count, because recursion
		duration *= 0.5 # Split time & range in half for subdivision,
		range    *= 0.5 # then pick random center point within range:
		midValue  = ((startValue + endValue - range) * 0.5 +
		             random.uniform(0.0, range))
		split(startValue, midValue, duration, range)
		split(midValue  , endValue, duration, range)
	else: # No more subdivisons, do iris motion...
		dv = endValue - startValue
		while True:
			dt = time.time() - startTime
			if dt >= duration: break
			v = startValue + dv * dt / duration
			if   v < PUPIL_MIN: v = PUPIL_MIN
			elif v > PUPIL_MAX: v = PUPIL_MAX
			frame(v) # Draw frame w/interim pupil scale value


# MAIN LOOP -- runs continuously -------------------------------------------

while True:

	if PUPIL_IN >= 0: # Pupil scale from sensor
		v = bonnet.channel[PUPIL_IN].value
		# If you need to calibrate PUPIL_MIN and MAX,
		# add a 'print v' here for testing.
		if   v < PUPIL_MIN: v = PUPIL_MIN
		elif v > PUPIL_MAX: v = PUPIL_MAX
		# Scale to 0.0 to 1.0:
		v = (v - PUPIL_MIN) / (PUPIL_MAX - PUPIL_MIN)
		if PUPIL_SMOOTH > 0:
			v = ((currentPupilScale * (PUPIL_SMOOTH - 1) + v) /
			     PUPIL_SMOOTH)
		frame(v)
	else: # Fractal auto pupil scale
		v = random.random()
		split(currentPupilScale, v, 4.0, 1.0)
	currentPupilScale = v
