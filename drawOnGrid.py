import numpy as np

"""
Draws a flat triangle (regarding the y axis) that has its base downwards
Return : list of coordinates of the grid representing the filled triangle
"""

def drawFlatBottomTriangle(v1, v2, v3, drawLastLine=True):  # assume v2.y = v3.y and v2.x < v3.x
    assert (v2[1] == v3[1]) and v1[1] >= v2[1]
    points = []
    line2 = drawLine(v1, v2)
    line3 = drawLine(v1, v3)
    for i in range(v3[1], v1[1] + 1):
        line2tmp = []
        while (line2 and line2[-1][1] == i):
            line2tmp.append(line2.pop())
        line3tmp = []
        while (line3 and line3[-1][1] == i):
            line3tmp.append(line3.pop())
        if (drawLastLine or i != v3[1]):
            el2 = min(line2tmp, key=lambda p: p[0])
            el3 = max(line3tmp, key=lambda p: p[0])
            points.extend(drawLine(el2, el3))
    return points


"""
Draws a flat triangle (regarding the y axis) that has its base upwards
Return : list of coordinates of the grid representing the filled triangle
"""
def drawFlatTopTriangle(v1, v2, v3):  # assume v2.y = v3.y and v2.x < v3.x
    assert (v2[1] == v3[1]) and v1[1] <= v2[1]
    points = []
    line2 = drawLine(v2, v1)
    line3 = drawLine(v3, v1)
    for i in range(v1[1], v2[1] + 1):
        line2tmp = []
        while (line2 and line2[-1][1] == i):
            line2tmp.append(line2.pop())
        line3tmp = []
        while (line3 and line3[-1][1] == i):
            line3tmp.append(line3.pop())
        el2 = min(line2tmp, key=lambda p: p[0])
        el3 = max(line3tmp, key=lambda p: p[0])
        points.extend(drawLine(el2, el3))
    return points

"""
Draws any triangle
Return : list of coordinates of the grid representing the filled triangle
"""

def drawTriangle(v1, v2, v3):
    vertices = [v1, v2, v3]
    vertices.sort(key=lambda v: v[1])
    if (vertices[0][1] == vertices[1][1]):  # bottom triangle
        return drawFlatBottomTriangle(vertices[2], vertices[1], vertices[0])
    elif (vertices[1][1] == vertices[2][1]):  # top triangle
        return drawFlatTopTriangle(vertices[0], vertices[1], vertices[2])
    else:
        (x1, y1) = vertices[0]
        (x2, y2) = vertices[1]
        (x3, y3) = vertices[2]
        (x4, y4) = (int(x3 + (y2 - y3) * (x1 - x3) * 1.0 / (y1 - y3)), y2)
        # print (x1, y1), (x2, y2), (x3, y3), (x4, y4)
        points = drawFlatTopTriangle((x1, y1), (x2, y2), (x4, y4))
        points.extend(drawFlatBottomTriangle((x3, y3), (x2, y2), (x4, y4), drawLastLine=False))
        return points

"""
Draws a line between 2 points, based on Bresenham's Line Algorithm
"""
def drawLine(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    Source : roguebasin.com
    """

    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

"""
Test function of drawTriangle
"""
def testDrawTriangle(v1, v2, v3):
    matrix = np.zeros(shape = (10, 10))
    triangle = drawTriangle(v1, v2, v3)
    for point in triangle:
        matrix[point[0]][point[1]] = 1
    matrix[v1[0]][v1[1]] = 2
    matrix[v2[0]][v2[1]] = 2
    matrix[v3[0]][v3[1]] = 2
    print v1, v2, v3
    print triangle
    assert len(set(triangle)) == len(triangle)
    print matrix

"""
testDrawTriangle((9,9), (9,9), (9, 9))
testDrawTriangle((9,9), (0,9), (0, 0))
testDrawTriangle((0,9), (0,0), (1, 8))
testDrawTriangle((5,0), (0,5), (6, 9))
testDrawTriangle((0,9), (0,0), (0, 0))
"""