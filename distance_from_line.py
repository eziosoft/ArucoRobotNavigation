import math


def dot(v, w):
    x, y, z = v
    X, Y, Z = w
    return x * X + y * Y + z * Z


def length(v):
    x, y, z = v
    return math.sqrt(x * x + y * y + z * z)


def vector(b, e):
    x, y, z = b
    X, Y, Z = e
    return (X - x, Y - y, Z - z)


def unit(v):
    x, y, z = v
    mag = length(v)
    return (x / mag, y / mag, z / mag)


def distanceV(p0, p1):
    return length(vector(p0, p1))


def scale(v, sc):
    x, y, z = v
    return (x * sc, y * sc, z * sc)


def add(v, w):
    x, y, z = v
    X, Y, Z = w
    return (x + X, y + Y, z + Z)


def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0 / line_len)
    t = dot(line_unitvec, pnt_vec_scaled)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distanceV(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist, nearest)


#https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return int(x), int(y)
