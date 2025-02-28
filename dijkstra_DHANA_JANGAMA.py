import numpy as np
import cv2
import heapq

# Constants for map dimensions and colors
CLEARANCE = 5  # Safety margin in millimeters (mm)
MAP_WIDTH, MAP_HEIGHT = 1200, 500  # Dimensions of the map in mm
OBSTACLE_COLOR = (255,255 , 255)  # Color for obstacles (red in BGR format)
SAFETY_MARGIN_COLOR = (245, 245, 245)  # Color for the safety margin around obstacles (black)
NAVIGABLE_AREA_COLOR = (0, 0, 0)  # Color for navigable areas (white)
VISITED_NODES_COLOR = (0, 0, 255)  # Color for nodes that have been explored (green)
ROUTE_COLOR = (0, 154, 0)  # Color for the final path (blue)
KEY_AREA_COLOR = (0, 0, 155)  # Specific color for key areas if necessary (red)
SEARCH_TRACE_COLOR = (154, 0, 0)  # Color for visualizing the search process (red)

# Movement directions and their associated costs
movement_strategies = {
    'UP': ('UP', (0, -1), 1),
    'DOWN': ('DOWN', (0, 1), 1),
    'LEFT': ('LEFT', (-1, 0), 1.0),
    'RIGHT': ('RIGHT', (1, 0), 1.0),
    'UP_LEFT': ('UP_LEFT', (-1, -1), 1.4),
    'UP_RIGHT': ('UP_RIGHT', (1, -1), 1.4),
    'DOWN_LEFT': ('DOWN_LEFT', (-1, 1), 1.4),
    'DOWN_RIGHT': ('DOWN_RIGHT', (1, 1), 1.4),
}

# Preparing the map for pathfinding
environment_map = np.full((MAP_HEIGHT, MAP_WIDTH, 3), NAVIGABLE_AREA_COLOR, dtype=np.uint8)

def outline_CLEARANCE(image, margin):
    # Defining boundaries for safety margins on the map
    cv2.rectangle(image, (0, 0), (MAP_WIDTH, margin), SAFETY_MARGIN_COLOR, -1)  # Top
    cv2.rectangle(image, (0, MAP_HEIGHT - margin), (MAP_WIDTH, MAP_HEIGHT), SAFETY_MARGIN_COLOR, -1)  # Bottom
    cv2.rectangle(image, (0, 0), (margin, MAP_HEIGHT), SAFETY_MARGIN_COLOR, -1)  # Left
    cv2.rectangle(image, (MAP_WIDTH - margin, 0), (MAP_WIDTH, MAP_HEIGHT), SAFETY_MARGIN_COLOR, -1)  # Right

outline_CLEARANCE(environment_map, CLEARANCE)

def sketch_obstacle_margin(image, obstacles, margin):
    for shape in obstacles:
        # Adjusting for buffer zone to avoid obstacles properly
        buffer_shape = [
            (max(shape[0][0] - margin, 0), max(shape[0][1] - margin, 0)),
            (min(shape[1][0] + margin, MAP_WIDTH), min(shape[1][1] + margin, MAP_HEIGHT))
        ]
        cv2.rectangle(image, buffer_shape[0], buffer_shape[1], SAFETY_MARGIN_COLOR, -1)  # Safety margin area
        cv2.rectangle(image, shape[0], shape[1], OBSTACLE_COLOR, -1)  # Actual obstacle

# Defining obstacles and applying buffer zones
complex_obstacle_shapes = [
    [(900, 50), (1100, 125)],  # C shape part
    [(1020, 50), (1100, 450)],  # Additional parts
    [(900, 375), (1100, 450)]
]

more_shapes = [
    [(100, 0), (175, 400)],  # Example rectangle
    [(275, 100), (350, 500)]  # Another example
]

sketch_obstacle_margin(environment_map, complex_obstacle_shapes + more_shapes, CLEARANCE)

def depict_hexagon_margin(image, center, length):
    # Generate points for hexagon with added safety margin
    margin_points = generate_hexagon(center, length + CLEARANCE)
    cv2.fillPoly(image, [margin_points], SAFETY_MARGIN_COLOR)  # Margin area
    inner_points = generate_hexagon(center, length)
    cv2.fillPoly(image, [inner_points], OBSTACLE_COLOR)  # Hexagon itself

def generate_hexagon(centre, side_length):
    angle_start = np.pi / 6  # Starting angle for hexagon vertices calculation
    return np.array([
        (int(centre[0] + np.cos(angle_start + np.pi / 3 * i) * side_length),
         int(centre[1] + np.sin(angle_start + np.pi / 3 * i) * side_length))
        for i in range(6)
    ], np.int32).reshape((-1, 1, 2))

hex_center = (650, 250)
hex_side_length = 150

depict_hexagon_margin(environment_map, hex_center, hex_side_length)

def map_boundary_check(x, y):
    return 0 <= x < MAP_WIDTH and 0 <= y < MAP_HEIGHT

def obstacle_presence_check(coord):
    x, y = coord
    if 0 <= x < MAP_WIDTH and 0 <= y < MAP_HEIGHT:
        pixel_value = environment_map[y, x]
        return np.array_equal(pixel_value, SAFETY_MARGIN_COLOR)
    return False

traveled_nodes = []

def shortest_path(start, destination):
    global traveled_nodes
    traveled_nodes = []
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))
    path_back = {}
    accumulated_cost = {start: 0}

    while priority_queue:
        current_cost, current_position = heapq.heappop(priority_queue)
        if current_position == destination:
            break

        for direction, (shift_x, shift_y), move_cost in movement_strategies.values():
            next_position = (current_position[0] + shift_x, current_position[1] + shift_y)
            if map_boundary_check(*next_position) and not obstacle_presence_check(next_position):
                cost_update = accumulated_cost[current_position] + move_cost
                if next_position not in accumulated_cost or cost_update < accumulated_cost[next_position]:
                    accumulated_cost[next_position] = cost_update
                    heapq.heappush(priority_queue, (cost_update, next_position))
                    path_back[next_position] = current_position
                    if not obstacle_presence_check(next_position):
                        traveled_nodes.append(next_position)

    return path_back, accumulated_cost

def reconstruct_route(origin, start, end):
    current = end
    path = []
    while current != start:
        path.append(current)
        current = origin[current]
    path.append(start)
    path.reverse()
    return path

def visualize_route(path, steps_taken):
    display_img = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8)

    for x in range(MAP_WIDTH):
        for y in range(MAP_HEIGHT):
            flip_y = MAP_HEIGHT - y - 1
            if obstacle_presence_check((x, y)):
                display_img[flip_y, x] = OBSTACLE_COLOR
            else:
                display_img[flip_y, x] = NAVIGABLE_AREA_COLOR

    for index, node in enumerate(steps_taken):
        flip_y = MAP_HEIGHT - node[1] - 1
        cv2.circle(display_img, (node[0], flip_y), 1, SEARCH_TRACE_COLOR, -1)
        if index % 100 == 0:
            flipped_img = cv2.flip(display_img, 0)
            cv2.imshow("Path Exploration", flipped_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    for step in path:
        flip_y = MAP_HEIGHT - step[1] - 1
        cv2.circle(display_img, (step[0], flip_y), 2, ROUTE_COLOR, -1)

    display_img = cv2.flip(display_img, 0)
    cv2.imshow("Final Path", display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def execute_navigation():
    start_x, start_y = map(int, input("Enter start coordinate (x y): ").split())
    end_x, end_y = map(int, input("Enter end coordinate (x y): ").split())

    start_coord = (start_x, MAP_HEIGHT - start_y)
    end_coord = (end_x, MAP_HEIGHT - end_y)

    path_origin, costs = shortest_path(start_coord, end_coord)
    if end_coord not in path_origin:
        print("Destination unreachable from starting point with current map configuration.")
        return

    final_route = reconstruct_route(path_origin, start_coord, end_coord)
    visualize_route(final_route, traveled_nodes)

if __name__ == '__main__':
    execute_navigation()
