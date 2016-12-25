import pi3d
import math
from svg.path import Path, parse_path

# Get artboard bounds (to use Illustrator terminology) from SVG DOM tree:
def getViewBox(root):
	for node in root.childNodes:
		if node.nodeType == node.ELEMENT_NODE:
			vb = getViewBox(node)
			if vb != None: return vb
			if node.tagName.lower() == "svg":
				vb = node.getAttribute("viewBox").split()
				return (float(vb[0]), float(vb[1]),
				        float(vb[2]), float(vb[3]))
	return None


# Search for and return a specific path (by name) in SVG DOM tree:
def getPath(root, id):
	for node in root.childNodes:
		if node.nodeType == node.ELEMENT_NODE:
			p = getPath(node, id)
			if p != None: return p
			if((node.tagName.lower() == "path") and
			   (node.getAttribute("id") == id)):
				return parse_path(node.getAttribute("d"))
	return None


# Convert SVG path to a 2D point list. Provide path, number of points,
# and whether or not this is a closed path (loop). For closed loops, the
# size of the point list returned is one element larger than the number of
# points passed, and the first and last elements will coincide.
def pathToPoints(path, numPoints, closed, reverse):
	points = []
	if numPoints < 2: numPoints  = 2
	if closed is True: div = float(numPoints)
	else:              div = float(numPoints - 1)
	for p in range(numPoints):
		if reverse is True: pt = path.point(1.0 - p / div, error=1e-5)
		else:               pt = path.point(      p / div, error=1e-5)
		points.append((pt.real, pt.imag))
	if closed is True: points.append(points[0])
	return points


# Combo wrapper for pathToPoints(getPath(...))
def getPoints(root, id, numPoints, closed, reverse):
	return pathToPoints(getPath(root, id), numPoints, closed, reverse)


# Scale a given 2D point list by normalizing to a given view box (returned
# by getViewBox()) then expanding to a given size centered on (0,0).
def scalePoints(p, vb, radius):
	for i, pt in enumerate(p): # Each point in path
		xx = ((p[i][0] - vb[0]) / vb[2] - 0.5) * radius *  2.0
		yy = ((p[i][1] - vb[1]) / vb[3] - 0.5) * radius * -2.0
		p[i] = (xx, yy)


# Interpolate between two 2D point lists, returning a new point list.
# Specify weighting (0.0 to 1.0) of second list.
# Lists should have same number of points; if not, lesser point count
# is used and the output may be weird.
def pointsInterp(points1, points2, p2weight):
	if   p2weight < 0.0: p2weight = 0.0
	elif p2weight > 1.0: p2weight = 1.0
	p1weight = 1.0 - p2weight
	points   = []
	np1      = len(points1)
	np2      = len(points2)
	if np2 < np1: np1 = np2
	if np1 < 1  : return None
	for p in range(np1):
		x = points1[p][0] * p1weight + points2[p][0] * p2weight
		y = points1[p][1] * p1weight + points2[p][1] * p2weight
		points.append((x, y))
	return points


# Return bounding rect of 2D point list
def pointsBounds(points):
	b = [ points[0][0], points[0][1], points[0][0], points[0][1] ]
	n = len(points)
	for p in range(1, n):
		if points[p][0] < b[0]: b[0] = points[p][0]
		if points[p][1] < b[1]: b[1] = points[p][1]
		if points[p][0] > b[2]: b[2] = points[p][0]
		if points[p][1] > b[3]: b[3] = points[p][1]
	return b # 4-element list: min X, min Y, max X, max Y


# This function rotates a model 90 degrees on the X axis and applies an
# offset to the texture map's U axis.  pi3d.Lathe() operates around the Y
# axis, but the eyes need symmetry around the Z axis and applying that
# transformation along with the eye rotation produced undesirable motion
# paths.  This is a hacky workaround.  It messes around with some pi3d
# data structures that it probably shouldn't, and could break with future
# releases of that code.
# vertices = buf[0,1,2]
# normals = buf[3,4,5]
# tex_coords = buf[6,7,8]
def reAxis(shape, texOffset):
	buf = shape.buf[0].array_buffer
	for i, v in enumerate(buf):
		# Rotate vertex
		tmp        = buf[i][1]
		buf[i][1]  = buf[i][2]
		buf[i][2]  = -tmp
		# Rotate normal
		tmp        = buf[i][4]
		buf[i][4]  = buf[i][5]
		buf[i][5]  = -tmp
		# Offset texture map on U axis
		buf[i][6] += texOffset



# Instead of making these so general-purpose, I might intentionally
# rig them to specifically handle the iris (closed shape) and eyelid
# (open shape) cases. Esp. since the iris is a weird case that'll need
# 3 paths (edge, open and closed)





# Given number of X and Y steps, generate boilerplate object with
# given texture coordinates and point indices. The initial vertex
# positions can be dummies (all 0,0,0) and will be reassigned with
# second function below.
# Usteps might be determined by passing in a pointlist instead,
# even though we're not using the coordinates yet, it'd provide some
# consistency and avoid trouble later.

# If it's an eyelid, add an extra row with V=0.0

def meshInit(uSteps, vSteps, closed, uOffset, vOffset, lid):
	verts = []
	tex   = []
	idx   = []
	norms = []
	if closed is True: uSteps += 1
	uDiv  = float(uSteps - 1)
	vDiv  = float(vSteps - 1)

	if lid is True: # Add extra row of vertices (with V=0) if eyelid
		for u in range(uSteps):
			verts.append((0,0,0))
			tex.append((u / uDiv + uOffset, vOffset))
			norms.append((0,0,-1))
		vRange = vSteps
	else:
		vRange = vSteps - 1

	for v in range(vSteps):
		v2 = vOffset + (v / vDiv) * (1.0 - vOffset * 2.0)
		for u in range(uSteps):
			verts.append((0,0,0))
			tex.append((u / uDiv + uOffset, v2))
			norms.append((0,0,-1))

	for v in range(vRange):
		for u in range(uSteps - 1):
			s = v * uSteps + u
 			idx.append((s+uSteps, s         , s+1     ))
 			idx.append((s+1     , s+uSteps+1, s+uSteps))

	shape = pi3d.Shape(None, None, "foo", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	  1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
	shape.buf = []
	shape.buf.append(pi3d.Buffer(shape, verts, tex, idx, norms, False))

	return shape


# Generate mesh between two point lists. U axis steps are determined
# by number of points, V axis determined by 'steps'
def pointsMesh(points0, points1, points2, steps, z, closed, flip=False):
	if steps < 2: steps = 2
	np1 = len(points1)
	np2 = len(points2)
	if np2 < np1: np1 = np2
	if np1 < 1  : return None

	verts = []

	if flip is True:
		if points0 is not None:
			for p in reversed(points0):
				verts.append((-p[0], p[1], 0))

		div = float(steps - 1)
		for y in range(steps):
			pList = pointsInterp(points1, points2, y / div)
			for x in reversed(range(np1)):
				verts.append((-pList[x][0], pList[x][1], z))
	else:
		if points0 is not None:
			for p in points0:
				verts.append((p[0], p[1], 0))

		div = float(steps - 1)
		for y in range(steps):
			pList = pointsInterp(points1, points2, y / div)
			for x in range(np1):
				verts.append((pList[x][0], pList[x][1], z))

	return verts


# This function determines the Z depth and angle-from-Z axis of an SVG
# feature (ostensibly a circle, polygonalized by getPoints()); for example,
# the depth of the iris, or the start and end angles for the curve that's
# lathed to form the sclera.  Pass point list and eye radius.
def zangle(points, r1):
	dx    = points[0][0]
	dy    = points[0][1]
	r2    = math.sqrt(dx * dx + dy * dy) # Radius of feature of interest
	z     = math.sqrt(r1 * r1 - r2 * r2)
	angle = math.atan2(r2, z) * 180.0 / math.pi

	return (z, angle)

