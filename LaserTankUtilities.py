import numpy as np
from math import pi

def rotate(points, angle):
    """
    A helper function for 2D rotations.
    Inputs:
        points: a nx2 numpy array
        angle: rotation angle in degrees
    Outputs:
        points: nx2 rotated array
    """
    ca = np.cos(angle*pi/180)
    sa = np.sin(angle*pi/180)
    R = np.array([[ca, -sa], [sa, ca]])  # positive is CCW
    R.shape += (1,)  # add dim for broadcasting over n points
    points = points.T
    points.shape = (1,) + points.shape  # 1x2xn
    points = (R*points).sum(axis=1).T  # do rotation and return original shape
    return points

def polygons_overlap(A, B):
    """
    Detect if two convex polygons overlap
    Inputs:
        A, B: a nx2 numpy array of vertices for each of the two polygons
    Outputs:
        overlaps: True if they overlap

    Searches for a line that can separate the two convex polygons completely.
    If such a line exists, the two polygons do not overlap. When the polygons
    do not overlap, infinitely many lines can be found to divide them, among
    which is always a set of lines parallel with one of the polygons' faces.
    Knowing this, we calculate a normal vector for each face and project the
    polygon vertices onto the normal vector. In this projected space, the
    face-parallel lines are respresented by a single point and they exist if
    the projections of the two polygons do not overlap in this 1D space.

    We can safely ignore sign and scaling on the normal vectors when testing
    for overlap. To appropriately use this data to resolve collisions, the
    normal vectors would have to be normalized.

    Useful references:
    codeproject.com "2D-Polygon-Collision-Detection" (https://goo.gl/5ykcTe)
    tutsplus.com "separating-axis-theorem" (https://goo.gl/5tdhRk)
    """
    overlaps = True
    edges = get_edges(A) + get_edges(B)
    normals = [np.array((y1-y2, x2-x1)) for (x1, y1), (x2, y2) in edges]
    # make unit length, with x > 0
    normals = [n * np.sign(n[0])/np.sqrt((n**2).sum()) for n in normals]
    # Any normal with the same slope corresponds to the same set of lines.
    unique_normals = []
    # Once we make the vectors unit length with nonnegative x, y uniquely
    # determines the slope, so we can check y to ensure uniqueness
    checked = []
    for n in normals:
        if n[1] not in checked:
            checked.append(n[1])
            unique_normals.append(n)
    # Search for a projection without any overlap
    for n in unique_normals:
        minA = maxA = minB = maxB = None
        for a in A:
            a = np.dot(a, n)
            if minA is None or a < minA:
                minA = a
            if maxA is None or a > maxA:
                maxA = a
        for b in B:
            b = np.dot(b, n)
            if minB is None or b < minB:
                minB = b
            if maxB is None or b > maxB:
                maxB = b
        if maxA < minB or maxB < minA:
            overlaps = False
            break
    # Only True if we searched every projection and found overlap for each
    return overlaps

def get_edges(points):
    next_points = points[1:]+points[:1]  # use [:1], not [0], to return a list
    edges = []
    for a, b in zip(points, next_points):
        edges.append(np.array((a, b)))
    return edges
