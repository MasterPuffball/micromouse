import cv2
import numpy as np
import matplotlib.pyplot as plt

# Constants
GRID_SIZE = 10
DIRECTIONS = ['N', 'E', 'S', 'W']
DIR_VECTORS = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}

# Preprocessing: grayscale, blur, edge detection
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    return closed

# Convert edge map to grid representation
def image_to_grid(edge_image, grid_size=GRID_SIZE):
    h, w = edge_image.shape
    cell_h, cell_w = h // grid_size, w // grid_size
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

    for i in range(grid_size):
        for j in range(grid_size):
            cell = edge_image[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
            borders = np.concatenate([
                            cell[:3, :].ravel(),         # top 3 rows
                            cell[-3:, :].ravel(),        # bottom 3 rows
                            cell[:, :3].ravel(),         # left 3 cols
                            cell[:, -3:].ravel()         # right 3 cols
                        ])

            if np.count_nonzero(borders) > 0:
                grid[i, j] = 1  # mark wall
    print(f"Grid cell ({i}, {j}) - wall: {grid[i,j]}")

    return grid
    
def crop_to_maze_bounds(edge_img):
    ys, xs = np.where(edge_img > 0)
    if len(xs) == 0 or len(ys) == 0:
        raise ValueError("No edges found in image.")
    min_x, max_x = np.min(xs), np.max(xs)
    min_y, max_y = np.min(ys), np.max(ys)
    return edge_img[min_y:max_y+1, min_x:max_x+1]


# Pathfinding using flood fill
def flood_fill(grid, goal):
    h, w = grid.shape
    dist = np.full((h, w), np.inf)
    dist[goal] = 0
    queue = [goal]
    while queue:
        x, y = queue.pop(0)
        for dx, dy in DIR_VECTORS.values():
            nx, ny = x+dx, y+dy
            if 0 <= nx < h and 0 <= ny < w and grid[nx, ny] == 0 and dist[nx, ny] == np.inf:
                dist[nx, ny] = dist[x, y] + 1
                queue.append((nx, ny))
    return dist

# Backtrack path from start using distance map
def trace_path(dist_map, start):
    path = [start]
    x, y = start
    while dist_map[x, y] != 0:
        min_val = dist_map[x, y]
        next_cell = None
        for dx, dy in DIR_VECTORS.values():
            nx, ny = x+dx, y+dy
            if 0 <= nx < dist_map.shape[0] and 0 <= ny < dist_map.shape[1]:
                if dist_map[nx, ny] < min_val:
                    min_val = dist_map[nx, ny]
                    next_cell = (nx, ny)
        if next_cell is None:
            break
        path.append(next_cell)
        x, y = next_cell
    return path

# Convert path to command string
def path_to_commands(path, start_dir='E'):
    commands = []
    current_dir = start_dir
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        next_dir = [d for d, v in DIR_VECTORS.items() if v == (dx, dy)][0]
        turns = (DIRECTIONS.index(next_dir) - DIRECTIONS.index(current_dir)) % 4
        if turns == 1:
            commands.append('r')
        elif turns == 3:
            commands.append('l')
        elif turns == 2:
            commands.extend(['r', 'r'])
        commands.append('f')
        current_dir = next_dir
    return ''.join(commands)

# Master function
def solve_maze_from_image(image_path, start=(0, 0), goal=(9, 9), start_dir='E'):
    image = cv2.imread(image_path)
    edge = preprocess_image(image)
    edge = crop_to_maze_bounds(edge)
    edge = cv2.resize(edge, (300, 300), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite("debug_edge_fresh.png", edge)

    grid = image_to_grid(edge)
    dist_map = flood_fill(grid, goal)
    path = trace_path(dist_map, start)
    commands = path_to_commands(path, start_dir)

    # Save grid visualization
    plt.imshow(grid, cmap="gray_r")
    plt.title("Detected Walls (Grid)")
    plt.grid(True)
    plt.xticks(range(GRID_SIZE))
    plt.yticks(range(GRID_SIZE))
    plt.savefig("debug_grid_fresh.png")
    plt.close()

    return commands