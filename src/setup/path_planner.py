import largestinteriorrectangle as lir
from PIL import Image, ImageDraw
import numpy
from math import hypot, floor
from python_tsp.exact import solve_tsp_dynamic_programming
import re
from pathfinding.core.grid import Grid
from pathfinding.finder.dijkstra import DijkstraFinder

DRIVEABLE_THRESH = 250
SIZE_THRESH = 400
S_RANGE = 12
MAP_SCALE = 0.05
X_OFFSET = -10
Y_OFFSET = -10


def plan_path(file_path):
    lpl_map = Image.open(file_path)
    x_max, y_max = lpl_map.size
    lir_map = numpy.zeros((x_max, y_max))
    for x in range(x_max):
        for y in range(y_max):
            if lpl_map.getpixel((x, y)) > DRIVEABLE_THRESH:
                lir_map[x][y] = 1
    lir_boolmap = numpy.array(lir_map, "bool")

    rectangles = find_rectangles(lir_boolmap)

    rectangles = split_rectangles(rectangles)
    centers = list(map(get_center_of_rectangle, rectangles))
    print("Centers done\n")

    distances = make_distance_matrix(centers, Grid(matrix=lir_map))
    order = solve_tsp_dynamic_programming(distances)
    print("ordered\n")
    print(f"order: {order}")

    path = aggregate_path(centers, order[0])
    return path, x_max, y_max


def find_rectangles(lir_boolmap):
    rectangles = []
    done = False
    while not done:
        rec = lir.lir(lir_boolmap)
        rec = (rec[1], rec[0], rec[3], rec[2])
        if rec[2] * rec[3] < SIZE_THRESH:
            done = True
            continue
        rectangles.append(rec)
        print("r")
        for x in range(rec[0], rec[0] + rec[2]):
            for y in range(rec[1], rec[1] + rec[3]):
                lir_boolmap[x][y] = False
    print(f"\nRectangles done: {len(rectangles)}\n")
    return rectangles


def split_rectangles(rectangles):
    return rectangles  # placeholder
    # new_rectangles = []
    # for rec in rectangles:
    #     if rec[2] > S_RANGE and rec[3] > S_RANGE:


def get_center_of_rectangle(rectangle):
    center = (rectangle[0] + (rectangle[2] // 2), rectangle[1] + (rectangle[3] // 2))
    return center


def make_distance_matrix(centers, grid: Grid):
    num_rec = len(centers)
    distance_matrix = numpy.zeros((num_rec, num_rec))
    finder = DijkstraFinder()
    cache = {}
    for x in range(num_rec):
        for y in range(num_rec):
            if (y, x) in cache:
                distance_matrix[x][y] = cache[(y, x)]
                print(f"dijkstra in cache{x * num_rec + y}/{num_rec * num_rec}")
            else:
                start = grid.node(centers[x][1], centers[x][0])
                end = grid.node(centers[y][1], centers[y][0])
                dist = len(finder.find_path(start, end, grid)[0])
                distance_matrix[x][y] = dist
                grid.cleanup()
                print(f"dijkstra{x * num_rec + y}/{num_rec * num_rec}")
                cache[(x,y)] = dist
    print("Distances matrixed\n")
    return distance_matrix


def make_distance_matrix_linear(centers):
    num_rec = len(centers)
    distance_matrix = numpy.zeros((num_rec, num_rec))
    for x in range(num_rec):
        for y in range(num_rec):
            # computes distance based purely on coordinates
            distance_matrix[x][y] = floor(hypot(abs(centers[x][0] - centers[y][0]), abs(centers[x][1] - centers[y][1])))
    print("Distances matrixed\n")
    return distance_matrix


def aggregate_path(points, order):
    path = []
    for i in order:
        path.append(points[i])
    print(f"Path aggregated: {path[:10]}")
    return path


def save_path(path, filename):
    file = open(file=filename, mode="w")
    if filename.endswith(".rospath"):
        file.write("<number>: <map_x>, <map_y>\n")
    else:
        file.write(f"<number>: <pixel_x>, <pixel_y>\n")
    i = 0
    for (x, y) in path:
        file.write(f"{i}: {x}, {y}\n")
        i += 1
    file.close()
    print("saved")


def draw_path(map_filename, path):
    lpl_map = Image.open(map_filename)
    draw = ImageDraw.Draw(lpl_map)
    draw.line(path, fill=128)
    filename = re.compile("(.+)\.pgm").search(map_filename).group(1)
    lpl_map.save(f"{filename}_with_path.png")


def read_pix_path(filename):
    file = open(filename)
    point_pattern = re.compile("(\d+): (\d+), (\d+)")
    path = []
    lines = file.readlines()
    assert lines[0] == "<number>: <pixel_x>, <pixel_y>\n"
    for line in lines[1:]:
        (number, x, y) = point_pattern.search(line).groups()
        path.append((int(x), int(y)))
    file.close()
    return path


def convert_pil_to_ros(pil_path, x_max, y_max):
    ros_path = []
    for (pil_x, pil_y) in pil_path:
        ros_x = (pil_x) * MAP_SCALE + X_OFFSET
        ros_y = (y_max - pil_y) * MAP_SCALE + Y_OFFSET
        ros_path.append((ros_x, ros_y))
    return ros_path


if __name__ == '__main__':
    map_path = "../../map/map.pgm"
    pixel_path, x_max, y_max = plan_path(map_path)
    ros_path = convert_pil_to_ros(pixel_path, x_max, y_max)
    # save_path(pixel_path, "../../path/lpl.path")
    save_path(ros_path, "../../path/lpl.rospath")
    # pixel_path = read_pix_path("../../path/lpl.path")
    # draw_path(map_path, pixel_path)

# grid = [[0, 0, 1, 0, 0, 0, 0, 0, 0],
#         [0, 0, 1, 0, 1, 1, 0, 0, 0],
#         [0, 0, 1, 1, 1, 1, 1, 0, 0],
#         [0, 0, 1, 1, 1, 1, 1, 1, 0],
#         [0, 0, 1, 1, 1, 1, 1, 1, 0],
#         [0, 1, 1, 1, 1, 1, 1, 0, 0],
#         [0, 0, 1, 1, 1, 0, 1, 0, 0],
#         [0, 0, 1, 1, 1, 1, 1, 0, 0],
#         [1, 1, 1, 1, 1, 1, 1, 0, 0],
#         [1, 1, 0, 0, 0, 1, 1, 1, 1],
#         [0, 0, 0, 0, 0, 0, 0, 0, 0]]
# bool_grid = numpy.array(grid,
#                         "bool")
#
# res = lir.lir(bool_grid)  # array([2, 2, 4, 7])
# for y in range(res[0], res[0] + res[2]):
#     for x in range(res[1], res[3] + res[1]):
#         grid[x][y] = 3
# print(numpy.array(grid))
