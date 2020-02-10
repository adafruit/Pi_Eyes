"""Utility functions used by the eyes code, related to 2D SGV paths and
   3D pi3d geometry."""

import math
import pi3d
from svg.path import parse_path

def get_view_box(root):
    """Get artboard bounds (to use Illustrator terminology)
       from SVG DOM tree:"""
    for node in root.childNodes:
        if node.nodeType == node.ELEMENT_NODE:
            view_box = get_view_box(node)
            if view_box:
                return view_box
            if node.tagName.lower() == "svg":
                view_box = node.getAttribute("viewBox").split()
                return (float(view_box[0]), float(view_box[1]),
                        float(view_box[2]), float(view_box[3]))
    return None


def get_path(root, path_name):
    """Search for and return a specific path (by name) in SVG DOM tree:"""
    for node in root.childNodes:
        if node.nodeType == node.ELEMENT_NODE:
            path = get_path(node, path_name)
            if path:
                return path
            if((node.tagName.lower() == "path") and
               (node.getAttribute("id") == path_name)):
                return parse_path(node.getAttribute("d"))
    return None


def path_to_points(path, num_points, closed, reverse):
    """Convert SVG path to a 2D point list. Provide path, number of points,
       and whether or not this is a closed path (loop). For closed loops,
       the size of the point list returned is one element larger than the
       number of points passed, and the first and last elements will
       coincide."""
    points = []
    num_points = max(num_points, 2)
    if closed:
        div = float(num_points)
    else:
        div = float(num_points - 1)
    for point_num in range(num_points):
        if reverse:
            point = path.point(1.0 - point_num / div, error=1e-5)
        else:
            point = path.point(point_num / div, error=1e-5)
        points.append((point.real, point.imag))
    if closed:
        points.append(points[0])
    return points


def get_points(root, path_name, num_points, closed, reverse):
    """Combo wrapper for path_to_points(get_path(...))."""
    return path_to_points(get_path(root, path_name),
                          num_points, closed, reverse)


def scale_points(points, view_box, radius):
    """Scale a given 2D point list by normalizing to a given view box
       (returned by get_view_box()) then expanding to a given size
       centered on (0,0)."""
    for point_num, _ in enumerate(points): # Index of each point in path
        points[point_num] = (((points[point_num][0] - view_box[0]) /
                              view_box[2] - 0.5) * radius *  2.0,
                             ((points[point_num][1] - view_box[1]) /
                              view_box[3] - 0.5) * radius * -2.0)


def points_interp(points1, points2, weight2):
    """Interpolate between two 2D point lists, returning a new point list.
       Specify weighting (0.0 to 1.0) of second list. Lists should have
       same number of points; if not, lesser point count is used and the
       output may be weird."""
    num_points = min(len(points1), len(points2))
    if num_points < 1:
        return None
    weight2 = min(max(0.0, weight2), 1.0)
    weight1 = 1.0 - weight2
    points = []
    for point_num in range(num_points):
        points.append(
            (points1[point_num][0] * weight1 + points2[point_num][0] * weight2,
             points1[point_num][1] * weight1 + points2[point_num][1] * weight2))
    return points


def points_bounds(points):
    """Return bounding rect of 2D point list (as 4-tuple of min X, min Y,
       max X, max Y)."""
    min_x, min_y, max_x, max_y = (points[0][0], points[0][1],
                                  points[0][0], points[0][1])
    for point in points[1:]:
        min_x = min(min_x, point[0])
        min_y = min(min_y, point[1])
        max_x = max(max_x, point[0])
        max_y = max(max_y, point[1])
    return (min_x, min_y, max_x, max_y)


def re_axis(shape, texture_offset):
    """Rotates a model 90 degrees on the X axis and applies an offset to
       the texture map's U axis. pi3d.Lathe() operates around the Y axis,
       but the eyes need symmetry around the Z axis and applying that
       transformation along with the eye rotation produced undesirable
       motion paths. This is a hacky workaround. It messes around with
       some pi3d data structures directly that it probably shouldn't,
       and might break with future releases of that code."""
    # vertices = buf[0,1,2]
    # normals = buf[3,4,5]
    # tex_coords = buf[6,7,8]
    abuf = shape.buf[0].array_buffer
    for vertex_num, _ in enumerate(abuf):
        # Rotate vertex
        abuf[vertex_num][1], abuf[vertex_num][2] = (
            abuf[vertex_num][2], -abuf[vertex_num][1])
        # Rotate normal
        abuf[vertex_num][4], abuf[vertex_num][5] = (
            abuf[vertex_num][5], -abuf[vertex_num][4])
        # Offset texture map on U axis
        abuf[vertex_num][6] += texture_offset



# Instead of making these so general-purpose, I might intentionally
# rig them to specifically handle the iris (closed shape) and eyelid
# (open shape) cases. Esp. since the iris is a weird case that'll need
# 3 paths (edge, open and closed)



def mesh_init(uv_steps, uv_offset, closed, lid):
    """Given number of U and V steps, generate boilerplate object with
       given texture coordinates and point indices. The initial vertex
       positions can be dummies (all 0,0,0) and will be reassigned later.
       If it's an eyelid, add an extra row with V=0.0."""
    # U steps might be determined by passing in a pointlist instead,
    # even though we're not using the coordinates yet, it'd provide some
    # consistency and avoid trouble later.
    verts = []
    tex = []
    idx = []
    norms = []
    if closed:
        uv_steps = (uv_steps[0] + 1, uv_steps[1])
    uv_div = (float(uv_steps[0] - 1), float(uv_steps[1] - 1))

    if lid: # Add extra row of vertices (with V=0) if eyelid
        for u_pos in range(uv_steps[0]):
            verts.append((0, 0, 0))
            tex.append((u_pos / uv_div[0] + uv_offset[0], uv_offset[1]))
            norms.append((0, 0, -1))
        v_range = uv_steps[1]
    else:
        v_range = uv_steps[1] - 1

    for v_pos in range(uv_steps[1]):
        v_pos_2 = (uv_offset[1] +
                   (v_pos / uv_div[1]) * (1.0 - uv_offset[1] * 2.0))
        for u_pos in range(uv_steps[0]):
            verts.append((0, 0, 0))
            tex.append((u_pos / uv_div[0] + uv_offset[0], v_pos_2))
            norms.append((0, 0, -1))

    for v_pos in range(v_range):
        for u_pos in range(uv_steps[0] - 1):
            pos = v_pos * uv_steps[0] + u_pos
            idx.append((pos + uv_steps[0], pos, pos + 1))
            idx.append((pos + 1, pos + uv_steps[0] + 1, pos + uv_steps[0]))

    shape = pi3d.Shape(None, None, "foo", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
    shape.buf = [pi3d.Buffer(shape, verts, tex, idx, norms, False)]

    return shape


def points_mesh(points, steps, z_coord, flip=False):
    """Generate mesh between two point lists. U axis steps are determined
       by number of points, V axis determined by 'steps'"""
    steps = max(steps, 2)
    num_points = min(len(points[1]), len(points[2]))
    if num_points < 1:
        return None

    verts = []

    if flip is True:
        if points[0]:
            for point in reversed(points[0]):
                verts.append((-point[0], point[1], z_coord))

        div = float(steps - 1)
        for y_index in range(steps):
            point_list = points_interp(points[1], points[2], y_index / div)
            for point in reversed(point_list):
                verts.append((-point[0], point[1], z_coord))
    else:
        if points[0]:
            for point in points[0]:
                verts.append((point[0], point[1], z_coord))

        div = float(steps - 1)
        for y_index in range(steps):
            point_list = points_interp(points[1], points[2], y_index / div)
            for point in point_list:
                verts.append((point[0], point[1], z_coord))

    return verts


def zangle(points, eye_radius):
    """Determines the Z depth and angle-from-Z axis of an SVG feature
       (ostensibly a circle, polygonalized by get_points()); for example,
       the depth of the iris, or the start and end angles for the curve
       that's lathed to form the sclera. Pass point list and eye radius."""
    xdist = points[0][0]
    ydist = points[0][1]
    radius = math.sqrt(xdist * xdist + ydist * ydist) # R of SVG feature
    z_depth = math.sqrt(eye_radius * eye_radius - radius * radius)
    angle = math.atan2(radius, eye_radius) * 180.0 / math.pi

    return (z_depth, angle)
