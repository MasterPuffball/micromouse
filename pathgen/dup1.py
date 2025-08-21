
import cv2
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import sys

# --- HSV Processing ---
def load_and_mask_hsv(image_path, hsv_range, resize_scale=0.25):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    img = cv2.resize(img, (int(img.shape[1] * resize_scale), int(img.shape[0] * resize_scale)))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_range['lower'])
    upper = np.array(hsv_range['upper'])
    mask = cv2.inRange(hsv, lower, upper)
    return mask, img

# --- Morphology ---
def clean_mask(mask):
    kernel_erode = np.ones((4, 4), np.uint8)
    kernel_dilate = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(mask, kernel_erode, iterations=5)
    dilated = cv2.dilate(eroded, kernel_dilate, iterations=4)
    return dilated
    
# Crop based on white pixels
def crop_to_outer_maze(mask, pad=25):
    ys, xs = np.where(mask == 255)
    if ys.size == 0 or xs.size == 0:
        raise ValueError("No white pixels found.")
    top = max(ys.min() - pad, 0)
    bottom = min(ys.max() + pad-70, mask.shape[0])
    left = max(xs.min() - pad + 170, 0)
    right = min(xs.max() + pad, mask.shape[1])
    return mask[top:bottom, left:right]

# Overlay the found path on the (cropped) masked image 
# floats
def cell_center(r, c, cell_h, cell_w):
    x = int(round(c * cell_w + 0.5 * cell_w))
    y = int(round(r * cell_h + 0.5 * cell_h))
    return (x, y)

def overlay_path_on_mask(cropped_mask, rows, cols, start_rc, start_dir, cmds, goal_rc=None):
    
    # Prepare canvas
    vis = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
    h, w = cropped_mask.shape
    cell_h, cell_w = h // rows, w // cols

    # Reasonable thickness relative to cell size
    thickness = max(2, min(cell_h, cell_w) // 5)

    # Simulate the movement to get the list of centres
    r, c = start_rc
    d = start_dir.upper()
    pts = [cell_center(r, c, cell_h, cell_w)]

    for cmd in cmds:
        if cmd == 'l':
            d = left_dir(d)
        elif cmd == 'r':
            d = right_dir(d)
        elif cmd == 'f':
            dr, dc = delta[d]
            r += dr
            c += dc
            pts.append(cell_center(r, c, cell_h, cell_w))

    # Draw polyline in green
    for i in range(1, len(pts)):
        cv2.line(vis, pts[i-1], pts[i], (0, 255, 0), thickness, lineType=cv2.LINE_AA)

    # Mark start (blue) and goal (red) for clarity
    if pts:
        cv2.circle(vis, pts[0], radius=max(2, thickness), color=(255, 0, 0), thickness=-3)
    if goal_rc is not None:
        gy, gx = goal_rc
        cv2.circle(vis, cell_center(gy, gx, cell_h, cell_w), radius=max(2, thickness), color=(0, 0, 255), thickness=-3)

    return vis



# --- Extract walls ---
def extract_maze_structure_centered(mask, rows, cols, threshold=0, strip_thickness=4):
    h, w = mask.shape
    cell_h = h // rows
    cell_w = w // cols
    maze = [[{'N': False, 'S': False, 'E': False, 'W': False} for _ in range(cols)] for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            y1, y2 = r * cell_h, (r + 1) * cell_h
            x1, x2 = c * cell_w, (c + 1) * cell_w
            cell = mask[y1:y2, x1:x2]
            cx, cy = cell_w // 2, cell_h // 2
            half = strip_thickness // 2
            if r > 0 and np.sum(cell[0:strip_thickness, cx-half:cx+half+1] == 0) > threshold:
                maze[r][c]['N'] = True
            if r < rows-1 and np.sum(cell[-strip_thickness:, cx-half:cx+half+1] == 0) > threshold:
                maze[r][c]['S'] = True
            if c > 0 and np.sum(cell[cy-half:cy+half+1, 0:strip_thickness] == 0) > threshold:
                maze[r][c]['W'] = True
            if c < cols-1 and np.sum(cell[cy-half:cy+half+1, -strip_thickness:] == 0) > threshold:
                maze[r][c]['E'] = True
    return maze

# --- ASCII Renderer ---
def generate_ascii_maze(maze):
    rows, cols = len(maze), len(maze[0])
    out = []
    for r in range(rows):
        top = "".join("+" + ("---" if maze[r][c]['N'] or r == 0 else "   ") for c in range(cols)) + "+"
        mid = "".join(("|" if maze[r][c]['W'] or c == 0 else " ") + "   " for c in range(cols)) + "|"
        out += [top, mid]
    bottom = "".join("+" + ("---" if maze[-1][c]['S'] or r == rows-1 else "   ") for c in range(cols)) + "+"
    out.append(bottom)
    return "\n".join(out)

# --- Pathfinding ---
DIRS = ['N', 'E', 'S', 'W']
delta = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
def left_dir(d): return DIRS[(DIRS.index(d) - 1) % 4]
def right_dir(d): return DIRS[(DIRS.index(d) + 1) % 4]
def opposite_dir(d):
    return {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[d]
def heuristic(r1, c1, r2, c2): return abs(r1 - r2) + abs(c1 - c2)

def solve_maze(maze, start, goal):
    sr, sc, sd = start
    gr, gc = goal
    visited = set()
    pq = PriorityQueue()
    pq.put((heuristic(sr, sc, gr, gc), 0, sr, sc, sd, []))
    while not pq.empty():
        _, g, r, c, d, path = pq.get()
        if (r, c, d) in visited:
            continue
        visited.add((r, c, d))
        if (r, c) == (gr, gc):
            return path
        dr, dc = delta[d]
        nr, nc = r + dr, c + dc
        if (
                0 <= nr < len(maze) and 0 <= nc < len(maze[0])
                and not maze[r][c][d]
                and not maze[nr][nc][opposite_dir(d)]
            ):
            pq.put((g + 1 + heuristic(nr, nc, gr, gc), g + 1, nr, nc, d, path + ['f']))
        pq.put((g + 1 + heuristic(r, c, gr, gc), g + 1, r, c, left_dir(d), path + ['l']))
        pq.put((g + 1 + heuristic(r, c, gr, gc), g + 1, r, c, right_dir(d), path + ['r']))
    return []

def solve_maze_from_image(image_path, start, goal, start_dir='E', rows=9, cols=9):
    hsv_values = {'lower':[0, 0, 138], 'upper': [179, 255, 255]}
    
    mask, _ = load_and_mask_hsv(image_path, hsv_values)
    cleaned = clean_mask(mask)
    cropped = crop_to_outer_maze(cleaned, pad=5)
    maze = extract_maze_structure_centered(cropped, rows=rows, cols=cols)
    
    print("ASCII Maze:\n")
    print(generate_ascii_maze(maze))

    start_full = (start[0], start[1], start_dir.upper())
    cmds = solve_maze(maze, start_full, goal)

    print("\nMovement Command String:")
    print("".join(cmds))

    # --- NEW: overlay green path on the cropped mask ---
    overlay = overlay_path_on_mask(
        cropped_mask=cropped,
        rows=rows, cols=cols,
        start_rc=(start[0], start[1]),
        start_dir=start_dir,
        cmds=cmds,
        goal_rc=(goal[0], goal[1])
    )

    # Show result
    plt.figure(figsize=(6,6))
    plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
    plt.title("Path overlay on masked image")
    plt.axis('off')
    plt.show()
