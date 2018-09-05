#!/usr/bin/python

# This is a hasty port of the Teensy eyes code to Python...all kludgey with
# an embarrassing number of globals in the frame() function and stuff.
# Needed to get SOMETHING working, can focus on improvements next.

import Adafruit_ADS1x15
import argparse
import math
import pi3d
import random
import thread
import time
import RPi.GPIO as GPIO
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from gfxutil import *

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
WINK_L_PIN      = 22    # GPIO pin for LEFT eye wink button
BLINK_PIN       = 23    # GPIO pin for blink button (BOTH eyes)
WINK_R_PIN      = 24    # GPIO pin for RIGHT eye wink button
AUTOBLINK       = True  # If True, eyes blink autonomously
CRAZY_EYES      = False # If True, each eye moves in different directions


# GPIO initialization ------------------------------------------------------

GPIO.setmode(GPIO.BCM)
if WINK_L_PIN >= 0: GPIO.setup(WINK_L_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
if BLINK_PIN  >= 0: GPIO.setup(BLINK_PIN , GPIO.IN, pull_up_down=GPIO.PUD_UP)
if WINK_R_PIN >= 0: GPIO.setup(WINK_R_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# ADC stuff ----------------------------------------------------------------

if JOYSTICK_X_IN >= 0 or JOYSTICK_Y_IN >= 0 or PUPIL_IN >= 0:
	adc      = Adafruit_ADS1x15.ADS1015()
	adcValue = [0] * 4
else:
	adc = None

# Because ADC reads are blocking operations, they normally would slow down
# the animation loop noticably, especially when reading multiple channels
# (even when using high data rate settings).  To avoid this, ADC channels
# are read in a separate thread and stored in the global list adcValue[],
# which the animation loop can read at its leisure (with immediate results,
# no slowdown).
def adcThread(adc, dest):
	while True:
		for i in range(len(dest)):
			# ADC input range is +- 4.096V
			# ADC output is -2048 to +2047
			# Analog inputs will be 0 to ~3.3V,
			# thus 0 to 1649-ish.  Read & clip:
			n = adc.read_adc(i, gain=1)
			if   n <    0: n =    0
			elif n > 1649: n = 1649
			dest[i] = n / 1649.0 # Store as 0.0 to 1.0
		time.sleep(0.01) # 100-ish Hz

# Start ADC sampling thread if needed:
if adc:
	thread.start_new_thread(adcThread, (adc, adcValue))


# Load SVG file, extract paths & convert to point lists --------------------

dom               = parse("graphics/eye.svg")
vb                = getViewBox(dom)
pupilMinPts       = getPoints(dom, "pupilMin"      , 32, True , True )
pupilMaxPts       = getPoints(dom, "pupilMax"      , 32, True , True )
irisPts           = getPoints(dom, "iris"          , 32, True , True )
scleraFrontPts    = getPoints(dom, "scleraFront"   ,  0, False, False)
scleraBackPts     = getPoints(dom, "scleraBack"    ,  0, False, False)
upperLidClosedPts = getPoints(dom, "upperLidClosed", 33, False, True )
upperLidOpenPts   = getPoints(dom, "upperLidOpen"  , 33, False, True )
upperLidEdgePts   = getPoints(dom, "upperLidEdge"  , 33, False, False)
lowerLidClosedPts = getPoints(dom, "lowerLidClosed", 33, False, False)
lowerLidOpenPts   = getPoints(dom, "lowerLidOpen"  , 33, False, False)
lowerLidEdgePts   = getPoints(dom, "lowerLidEdge"  , 33, False, False)


# Set up display and initialize pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

# eyeRadius is the size, in pixels, at which the whole eye will be rendered
# onscreen.  eyePosition, also pixels, is the offset (left or right) from
# the center point of the screen to the center of each eye.  This geometry
# is explained more in-depth in fbx2.c.
eyePosition = DISPLAY.width / 4
eyeRadius   = 128  # Default; use 240 for IPS screens

parser = argparse.ArgumentParser()
parser.add_argument("--radius", type=int)
args = parser.parse_args()
if args.radius:
	eyeRadius = args.radius


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
scalePoints(pupilMinPts      , vb, eyeRadius)
scalePoints(pupilMaxPts      , vb, eyeRadius)
scalePoints(irisPts          , vb, eyeRadius)
scalePoints(scleraFrontPts   , vb, eyeRadius)
scalePoints(scleraBackPts    , vb, eyeRadius)
scalePoints(upperLidClosedPts, vb, eyeRadius)
scalePoints(upperLidOpenPts  , vb, eyeRadius)
scalePoints(upperLidEdgePts  , vb, eyeRadius)
scalePoints(lowerLidClosedPts, vb, eyeRadius)
scalePoints(lowerLidOpenPts  , vb, eyeRadius)
scalePoints(lowerLidEdgePts  , vb, eyeRadius)

# Regenerating flexible object geometry (such as eyelids during blinks, or
# iris during pupil dilation) is CPU intensive, can noticably slow things
# down, especially on single-core boards.  To reduce this load somewhat,
# determine a size change threshold below which regeneration will not occur;
# roughly equal to 1/4 pixel, since 4x4 area sampling is used.

# Determine change in pupil size to trigger iris geometry regen
irisRegenThreshold = 0.0
a = pointsBounds(pupilMinPts) # Bounds of pupil at min size (in pixels)
b = pointsBounds(pupilMaxPts) # " at max size
maxDist = max(abs(a[0] - b[0]), abs(a[1] - b[1]), # Determine distance of max
              abs(a[2] - b[2]), abs(a[3] - b[3])) # variance around each edge
# maxDist is motion range in pixels as pupil scales between 0.0 and 1.0.
# 1.0 / maxDist is one pixel's worth of scale range.  Need 1/4 that...
if maxDist > 0: irisRegenThreshold = 0.25 / maxDist

# Determine change in eyelid values needed to trigger geometry regen.
# This is done a little differently than the pupils...instead of bounds,
# the distance between the middle points of the open and closed eyelid
# paths is evaluated, then similar 1/4 pixel threshold is determined.
upperLidRegenThreshold = 0.0
lowerLidRegenThreshold = 0.0
p1 = upperLidOpenPts[len(upperLidOpenPts) / 2]
p2 = upperLidClosedPts[len(upperLidClosedPts) / 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: upperLidRegenThreshold = 0.25 / math.sqrt(d)
p1 = lowerLidOpenPts[len(lowerLidOpenPts) / 2]
p2 = lowerLidClosedPts[len(lowerLidClosedPts) / 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: lowerLidRegenThreshold = 0.25 / math.sqrt(d)

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

rightUpperEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
rightUpperEyelid.set_textures([lidMap])
rightUpperEyelid.set_shader(shader)
rightLowerEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
rightLowerEyelid.set_textures([lidMap])
rightLowerEyelid.set_shader(shader)

# Generate scleras for each eye...start with a 2D shape for lathing...
angle1 = zangle(scleraFrontPts, eyeRadius)[1] # Sclera front angle
angle2 = zangle(scleraBackPts , eyeRadius)[1] # " back angle
aRange = 180 - angle1 - angle2
pts    = []
for i in range(24):
	ca, sa = pi3d.Utility.from_polar((90 - angle1) - aRange * i / 23)
	pts.append((ca * eyeRadius, sa * eyeRadius))

# Scleras are generated independently (object isn't re-used) so each
# may have a different image map (heterochromia, corneal scar, or the
# same image map can be offset on one so the repetition isn't obvious).
leftEye = pi3d.Lathe(path=pts, sides=64)
leftEye.set_textures([scleraMap])
leftEye.set_shader(shader)
reAxis(leftEye, 0)
rightEye = pi3d.Lathe(path=pts, sides=64)
rightEye.set_textures([scleraMap])
rightEye.set_shader(shader)
reAxis(rightEye, 0.5) # Image map offset = 180 degree rotation


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

startXR      = random.uniform(-30.0, 30.0)
n            = math.sqrt(900.0 - startX * startX)
startYR      = random.uniform(-n, n)
destXR       = startXR
destYR       = startYR
curXR        = startXR
curYR        = startYR
moveDurationR = random.uniform(0.075, 0.175)
holdDurationR = random.uniform(0.1, 1.1)
startTimeR    = 0.0
isMovingR     = False

frames        = 0
beginningTime = time.time()

rightEye.positionX(-eyePosition)
rightIris.positionX(-eyePosition)
rightUpperEyelid.positionX(-eyePosition)
rightUpperEyelid.positionZ(-eyeRadius - 42)
rightLowerEyelid.positionX(-eyePosition)
rightLowerEyelid.positionZ(-eyeRadius - 42)

leftEye.positionX(eyePosition)
leftIris.positionX(eyePosition)
leftUpperEyelid.positionX(eyePosition)
leftUpperEyelid.positionZ(-eyeRadius - 42)
leftLowerEyelid.positionX(eyePosition)
leftLowerEyelid.positionZ(-eyeRadius - 42)

currentPupilScale       =  0.5
prevPupilScale          = -1.0 # Force regen on first frame
prevLeftUpperLidWeight  = 0.5
prevLeftLowerLidWeight  = 0.5
prevRightUpperLidWeight = 0.5
prevRightLowerLidWeight = 0.5
prevLeftUpperLidPts  = pointsInterp(upperLidOpenPts, upperLidClosedPts, 0.5)
prevLeftLowerLidPts  = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 0.5)
prevRightUpperLidPts = pointsInterp(upperLidOpenPts, upperLidClosedPts, 0.5)
prevRightLowerLidPts = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 0.5)

luRegen = True
llRegen = True
ruRegen = True
rlRegen = True

timeOfLastBlink = 0.0
timeToNextBlink = 1.0
# These are per-eye (left, right) to allow winking:
blinkStateLeft      = 0 # NOBLINK
blinkStateRight     = 0
blinkDurationLeft   = 0.1
blinkDurationRight  = 0.1
blinkStartTimeLeft  = 0
blinkStartTimeRight = 0

trackingPos = 0.3
trackingPosR = 0.3

# Generate one frame of imagery
def frame(p):

	global startX, startY, destX, destY, curX, curY
	global startXR, startYR, destXR, destYR, curXR, curYR
	global moveDuration, holdDuration, startTime, isMoving
	global moveDurationR, holdDurationR, startTimeR, isMovingR
	global frames
	global leftIris, rightIris
	global pupilMinPts, pupilMaxPts, irisPts, irisZ
	global leftEye, rightEye
	global leftUpperEyelid, leftLowerEyelid, rightUpperEyelid, rightLowerEyelid
	global upperLidOpenPts, upperLidClosedPts, lowerLidOpenPts, lowerLidClosedPts
	global upperLidEdgePts, lowerLidEdgePts
	global prevLeftUpperLidPts, prevLeftLowerLidPts, prevRightUpperLidPts, prevRightLowerLidPts
	global leftUpperEyelid, leftLowerEyelid, rightUpperEyelid, rightLowerEyelid
	global prevLeftUpperLidWeight, prevLeftLowerLidWeight, prevRightUpperLidWeight, prevRightLowerLidWeight
	global prevPupilScale
	global irisRegenThreshold, upperLidRegenThreshold, lowerLidRegenThreshold
	global luRegen, llRegen, ruRegen, rlRegen
	global timeOfLastBlink, timeToNextBlink
	global blinkStateLeft, blinkStateRight
	global blinkDurationLeft, blinkDurationRight
	global blinkStartTimeLeft, blinkStartTimeRight
	global trackingPos
	global trackingPosR

	DISPLAY.loop_running()

	now = time.time()
	dt  = now - startTime
	dtR  = now - startTimeR

	frames += 1
#	if(now > beginningTime):
#		print(frames/(now-beginningTime))

	if JOYSTICK_X_IN >= 0 and JOYSTICK_Y_IN >= 0:
		# Eye position from analog inputs
		curX = adcValue[JOYSTICK_X_IN]
		curY = adcValue[JOYSTICK_Y_IN]
		if JOYSTICK_X_FLIP: curX = 1.0 - curX
		if JOYSTICK_Y_FLIP: curY = 1.0 - curY
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
				holdDuration = random.uniform(0.1, 1.1)
				startTime    = now
				isMoving     = False
		else:
			if dt >= holdDuration:
				destX        = random.uniform(-30.0, 30.0)
				n            = math.sqrt(900.0 - destX * destX)
				destY        = random.uniform(-n, n)
				moveDuration = random.uniform(0.075, 0.175)
				startTime    = now
				isMoving     = True

		# repeat for other eye if CRAZY_EYES
        if CRAZY_EYES:
            if isMovingR == True:
                if dtR <= moveDurationR:
                    scale        = (now - startTimeR) / moveDurationR
                    # Ease in/out curve: 3*t^2-2*t^3
                    scale = 3.0 * scale * scale - 2.0 * scale * scale * scale
                    curXR        = startXR + (destXR - startXR) * scale
                    curYR        = startYR + (destYR - startYR) * scale
                else:
                    startXR      = destXR
                    startYR      = destYR
                    curXR        = destXR
                    curYR        = destYR
                    holdDurationR = random.uniform(0.1, 1.1)
                    startTimeR    = now
                    isMovingR     = False
            else:
                if dtR >= holdDurationR:
                    destXR        = random.uniform(-30.0, 30.0)
                    n             = math.sqrt(900.0 - destXR * destXR)
                    destYR        = random.uniform(-n, n)
                    moveDurationR = random.uniform(0.075, 0.175)
                    startTimeR    = now
                    isMovingR     = True

	# Regenerate iris geometry only if size changed by >= 1/4 pixel
	if abs(p - prevPupilScale) >= irisRegenThreshold:
		# Interpolate points between min and max pupil sizes
		interPupil = pointsInterp(pupilMinPts, pupilMaxPts, p)
		# Generate mesh between interpolated pupil and iris bounds
		mesh = pointsMesh(None, interPupil, irisPts, 4, -irisZ, True)
		# Assign to both eyes
		leftIris.re_init(pts=mesh)
		rightIris.re_init(pts=mesh)
		prevPupilScale = p

	# Eyelid WIP

	if AUTOBLINK and (now - timeOfLastBlink) >= timeToNextBlink:
		timeOfLastBlink = now
		duration        = random.uniform(0.035, 0.06)
		if blinkStateLeft != 1:
			blinkStateLeft     = 1 # ENBLINK
			blinkStartTimeLeft = now
			blinkDurationLeft  = duration
		if blinkStateRight != 1:
			blinkStateRight     = 1 # ENBLINK
			blinkStartTimeRight = now
			blinkDurationRight  = duration
		timeToNextBlink = duration * 3 + random.uniform(0.0, 4.0)

	if blinkStateLeft: # Left eye currently winking/blinking?
		# Check if blink time has elapsed...
		if (now - blinkStartTimeLeft) >= blinkDurationLeft:
			# Yes...increment blink state, unless...
			if (blinkStateLeft == 1 and # Enblinking and...
			    ((BLINK_PIN >= 0 and    # blink pin held, or...
			      GPIO.input(BLINK_PIN) == GPIO.LOW) or
			    (WINK_L_PIN >= 0 and    # wink pin held
			      GPIO.input(WINK_L_PIN) == GPIO.LOW))):
				# Don't advance yet; eye is held closed
				pass
			else:
				blinkStateLeft += 1
				if blinkStateLeft > 2:
					blinkStateLeft = 0 # NOBLINK
				else:
					blinkDurationLeft *= 2.0
					blinkStartTimeLeft = now
	else:
		if WINK_L_PIN >= 0 and GPIO.input(WINK_L_PIN) == GPIO.LOW:
			blinkStateLeft     = 1 # ENBLINK
			blinkStartTimeLeft = now
			blinkDurationLeft  = random.uniform(0.035, 0.06)

	if blinkStateRight: # Right eye currently winking/blinking?
		# Check if blink time has elapsed...
		if (now - blinkStartTimeRight) >= blinkDurationRight:
			# Yes...increment blink state, unless...
			if (blinkStateRight == 1 and # Enblinking and...
			    ((BLINK_PIN >= 0 and    # blink pin held, or...
			      GPIO.input(BLINK_PIN) == GPIO.LOW) or
			    (WINK_R_PIN >= 0 and    # wink pin held
			      GPIO.input(WINK_R_PIN) == GPIO.LOW))):
				# Don't advance yet; eye is held closed
				pass
			else:
				blinkStateRight += 1
				if blinkStateRight > 2:
					blinkStateRight = 0 # NOBLINK
				else:
					blinkDurationRight *= 2.0
					blinkStartTimeRight = now
	else:
		if WINK_R_PIN >= 0 and GPIO.input(WINK_R_PIN) == GPIO.LOW:
			blinkStateRight     = 1 # ENBLINK
			blinkStartTimeRight = now
			blinkDurationRight  = random.uniform(0.035, 0.06)

	if BLINK_PIN >= 0 and GPIO.input(BLINK_PIN) == GPIO.LOW:
		duration = random.uniform(0.035, 0.06)
		if blinkStateLeft == 0:
			blinkStateLeft     = 1
			blinkStartTimeLeft = now
			blinkDurationLeft  = duration
		if blinkStateRight == 0:
			blinkStateRight     = 1
			blinkStartTimeRight = now
			blinkDurationRight  = duration

	if TRACKING:
		n = 0.4 - curY / 60.0
		if   n < 0.0: n = 0.0
		elif n > 1.0: n = 1.0
		trackingPos = (trackingPos * 3.0 + n) * 0.25
		if CRAZY_EYES:
			n = 0.4 - curYR / 60.0
			if   n < 0.0: n = 0.0
			elif n > 1.0: n = 1.0
			trackingPosR = (trackingPosR * 3.0 + n) * 0.25

	if blinkStateLeft:
		n = (now - blinkStartTimeLeft) / blinkDurationLeft
		if n > 1.0: n = 1.0
		if blinkStateLeft == 2: n = 1.0 - n
	else:
		n = 0.0
	newLeftUpperLidWeight = trackingPos + (n * (1.0 - trackingPos))
	newLeftLowerLidWeight = (1.0 - trackingPos) + (n * trackingPos)

	if blinkStateRight:
		n = (now - blinkStartTimeRight) / blinkDurationRight
		if n > 1.0: n = 1.0
		if blinkStateRight == 2: n = 1.0 - n
	else:
		n = 0.0
	if CRAZY_EYES:
		newRightUpperLidWeight = trackingPosR + (n * (1.0 - trackingPosR))
		newRightLowerLidWeight = (1.0 - trackingPosR) + (n * trackingPosR)
	else:
		newRightUpperLidWeight = trackingPos + (n * (1.0 - trackingPos))
		newRightLowerLidWeight = (1.0 - trackingPos) + (n * trackingPos)

	if (luRegen or (abs(newLeftUpperLidWeight - prevLeftUpperLidWeight) >=
	  upperLidRegenThreshold)):
		newLeftUpperLidPts = pointsInterp(upperLidOpenPts,
		  upperLidClosedPts, newLeftUpperLidWeight)
		if newLeftUpperLidWeight > prevLeftUpperLidWeight:
			leftUpperEyelid.re_init(pts=pointsMesh(
			  upperLidEdgePts, prevLeftUpperLidPts,
			  newLeftUpperLidPts, 5, 0, False))
		else:
			leftUpperEyelid.re_init(pts=pointsMesh(
			  upperLidEdgePts, newLeftUpperLidPts,
			  prevLeftUpperLidPts, 5, 0, False))
		prevLeftUpperLidPts    = newLeftUpperLidPts
		prevLeftUpperLidWeight = newLeftUpperLidWeight
		luRegen = True
	else:
		luRegen = False

	if (llRegen or (abs(newLeftLowerLidWeight - prevLeftLowerLidWeight) >=
	  lowerLidRegenThreshold)):
		newLeftLowerLidPts = pointsInterp(lowerLidOpenPts,
		  lowerLidClosedPts, newLeftLowerLidWeight)
		if newLeftLowerLidWeight > prevLeftLowerLidWeight:
			leftLowerEyelid.re_init(pts=pointsMesh(
			  lowerLidEdgePts, prevLeftLowerLidPts,
			  newLeftLowerLidPts, 5, 0, False))
		else:
			leftLowerEyelid.re_init(pts=pointsMesh(
			  lowerLidEdgePts, newLeftLowerLidPts,
			  prevLeftLowerLidPts, 5, 0, False))
		prevLeftLowerLidWeight = newLeftLowerLidWeight
		prevLeftLowerLidPts    = newLeftLowerLidPts
		llRegen = True
	else:
		llRegen = False

	if (ruRegen or (abs(newRightUpperLidWeight - prevRightUpperLidWeight) >=
	  upperLidRegenThreshold)):
		newRightUpperLidPts = pointsInterp(upperLidOpenPts,
		  upperLidClosedPts, newRightUpperLidWeight)
		if newRightUpperLidWeight > prevRightUpperLidWeight:
			rightUpperEyelid.re_init(pts=pointsMesh(
			  upperLidEdgePts, prevRightUpperLidPts,
			  newRightUpperLidPts, 5, 0, False, True))
		else:
			rightUpperEyelid.re_init(pts=pointsMesh(
			  upperLidEdgePts, newRightUpperLidPts,
			  prevRightUpperLidPts, 5, 0, False, True))
		prevRightUpperLidWeight = newRightUpperLidWeight
		prevRightUpperLidPts    = newRightUpperLidPts
		ruRegen = True
	else:
		ruRegen = False

	if (rlRegen or (abs(newRightLowerLidWeight - prevRightLowerLidWeight) >=
	  lowerLidRegenThreshold)):
		newRightLowerLidPts = pointsInterp(lowerLidOpenPts,
		  lowerLidClosedPts, newRightLowerLidWeight)
		if newRightLowerLidWeight > prevRightLowerLidWeight:
			rightLowerEyelid.re_init(pts=pointsMesh(
			  lowerLidEdgePts, prevRightLowerLidPts,
			  newRightLowerLidPts, 5, 0, False, True))
		else:
			rightLowerEyelid.re_init(pts=pointsMesh(
			  lowerLidEdgePts, newRightLowerLidPts,
			  prevRightLowerLidPts, 5, 0, False, True))
		prevRightLowerLidWeight = newRightLowerLidWeight
		prevRightLowerLidPts    = newRightLowerLidPts
		rlRegen = True
	else:
		rlRegen = False

	convergence = 2.0

	# Right eye (on screen left)
	if CRAZY_EYES:
		rightIris.rotateToX(curYR)
		rightIris.rotateToY(curXR - convergence)
		rightIris.draw()
		rightEye.rotateToX(curYR)
		rightEye.rotateToY(curXR - convergence)
	else:
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

	leftUpperEyelid.draw()
	leftLowerEyelid.draw()
	rightUpperEyelid.draw()
	rightLowerEyelid.draw()

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
		v = adcValue[PUPIL_IN]
		if PUPIL_IN_FLIP: v = 1.0 - v
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
