
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
    eroded = cv2.erode(mask, kernel_erode, iterations=4)
    dilated = cv2.dilate(eroded, kernel_dilate, iterations=4)
    return dilated
    
# Crop based on white pixels
def crop_to_outer_maze(mask, pad=25):
    ys, xs = np.where(mask == 255)
    if ys.size == 0 or xs.size == 0:
        raise ValueError("No white pixels found.")
    top = max(ys.min() - pad, 0)
    bottom = min(ys.max() + pad, mask.shape[0])
    left = max(xs.min() - pad+110, 0)
    right = min(xs.max() + pad, mask.shape[1])
    return mask[top:bottom, left:right]

# Overlay the found path on the (cropped) masked image 
# floats
def cell_center(r, c, cell_h, cell_w):
    x = int(round(c * cell_w + 0.5 * cell_w))
    y = int(round(r * cell_h + 0.5 * cell_h))
    return (x, y)

def overlay_path_on_mask(cropped_mask, rows, cols, start_rc, start_dir, cmds, goal_rc=None):
    vis = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
    h, w = cropped_mask.shape
    cell_h, cell_w = h / rows, w / cols  # <-- floats

    thickness = max(2, int(min(cell_h, cell_w) // 5))

    r, c = start_rc
    d = start_dir.upper()
    pts = [cell_center(r, c, cell_h, cell_w)]
    for cmd in cmds:
        if cmd == 'l': d = left_dir(d)
        elif cmd == 'r': d = right_dir(d)
        elif cmd == 'f':
            dr, dc = delta[d]
            r += dr; c += dc
            pts.append(cell_center(r, c, cell_h, cell_w))

    if len(pts) >= 2:
        cv2.polylines(vis, [np.asarray(pts, np.int32)], False, (0,255,0), thickness, cv2.LINE_AA)

    if pts:
        cv2.circle(vis, pts[0], max(2, thickness), (255,0,0), -1)  # filled
    if goal_rc is not None:
        gy, gx = goal_rc
        cv2.circle(vis, cell_center(gy, gx, cell_h, cell_w), max(2, thickness), (0,0,255), -1)
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
# ---------- path utils ----------
def follow_cmds_to_cells(cmds, start_full):
    r, c, d = start_full
    cells = [(r, c)]
    for cmd in cmds:
        if cmd == 'l': d = left_dir(d)
        elif cmd == 'r': d = right_dir(d)
        elif cmd == 'f':
            dr, dc = delta[d]
            r, c = r + dr, c + dc
            cells.append((r, c))
    return cells

def cells_to_pixel_centres(cells, img_h, img_w, rows, cols):
    ch, cw = img_h / rows, img_w / cols
    pts = []
    for r, c in cells:
        x = int(round(c * cw + 0.5 * cw))
        y = int(round(r * ch + 0.5 * ch))
        pts.append((x, y))
    return pts

# ---------- RRT* in an ROI on a binary mask (255=free, 0=obstacle) ----------
from collections import defaultdict
import numpy as np

def rrt_star(mask_free, start_xy, goal_xy, max_iters=1200, step=16,
             goal_tol=20, goal_bias=0.25, rewire_radius=40,
             col_stride=2, bin_size=None):
    """
    mask_free: uint8, 255=free, 0=blocked (ROI)
    col_stride: sample every N pixels for collision checking (>=1)
    bin_size: spatial hash bin size in px (defaults to rewire_radius)
    """
    import math
    h, w = mask_free.shape
    start = np.array(start_xy, dtype=np.float32)
    goal  = np.array(goal_xy, dtype=np.float32)

    # --- spatial bins for ~O(1) neighbor queries
    if bin_size is None: bin_size = max(8, int(rewire_radius))
    bins = defaultdict(list)
    def bkey(pt): return (int(pt[0] // bin_size), int(pt[1] // bin_size))
    def add_to_bins(idx, pt): bins[bkey(pt)].append(idx)

    nodes = [start]
    parent = [-1]
    cost   = [0.0]
    add_to_bins(0, start)

    def sample():
        if np.random.rand() < goal_bias:
            return goal.copy()
        return np.array([np.random.uniform(0, w-1), np.random.uniform(0, h-1)], dtype=np.float32)

    def neighbors_approx(pt, radius):
        # gather nodes from bins overlapping the radius box
        bx, by = int(pt[0] // bin_size), int(pt[1] // bin_size)
        r_bins = int(math.ceil(radius / bin_size)) + 1
        cand = []
        for dy in range(-r_bins, r_bins+1):
            for dx in range(-r_bins, r_bins+1):
                cand.extend(bins.get((bx+dx, by+dy), []))
        if not cand:
            return np.array([], dtype=int)
        cand = np.array(cand, dtype=int)
        pts = np.asarray(nodes)[cand]
        d2 = np.sum((pts - pt)**2, axis=1)
        return cand[d2 <= radius*radius]

    def nearest_approx(pt):
        # search nearby bins, expand if empty
        bx, by = int(pt[0] // bin_size), int(pt[1] // bin_size)
        for r in range(0, 4):  # expand up to 4 rings
            cand = []
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    cand.extend(bins.get((bx+dx, by+dy), []))
            if cand:
                cand = np.array(cand, dtype=int)
                pts = np.asarray(nodes)[cand]
                i = int(np.argmin(np.sum((pts - pt)**2, axis=1)))
                return int(cand[i])
        # fallback: full scan (rare)
        pts = np.asarray(nodes)
        return int(np.argmin(np.sum((pts - pt)**2, axis=1)))

    def steer(p, q):
        v = q - p
        L = np.linalg.norm(v)
        if L <= 1e-9: return p.copy()
        if L <= step: return q.copy()
        return p + (v / L) * step

    def collision_free(p, q):
        L = max(1.0, np.linalg.norm(q - p))
        n = int(L // max(1, col_stride)) + 1
        xs = np.linspace(p[0], q[0], n).astype(int).clip(0, w-1)
        ys = np.linspace(p[1], q[1], n).astype(int).clip(0, h-1)
        return np.all(mask_free[ys, xs] > 0)

    def snap_to_free(pt, r=5):
        x, y = int(pt[0]), int(pt[1])
        if 0 <= x < w and 0 <= y < h and mask_free[y, x] > 0:
            return pt
        for rad in range(1, r+1):
            ys, xs = np.ogrid[-rad:rad+1, -rad:rad+1]
            disk = xs**2 + ys**2 <= rad*rad
            Y, X = np.where(disk)
            for dy, dx in zip(Y - rad, X - rad):
                yy, xx = (y + int(dy), x + int(dx))
                if 0 <= xx < w and 0 <= yy < h and mask_free[yy, xx] > 0:
                    return np.array([xx, yy], dtype=np.float32)
        return pt

    start = snap_to_free(start)
    goal  = snap_to_free(goal)

    for it in range(max_iters):
        q_rand = sample()
        i_near = nearest_approx(q_rand)
        q_new  = steer(nodes[i_near], q_rand)
        if not collision_free(nodes[i_near], q_new):
            continue

        # choose best parent among neighbors in rewire_radius
        neigh = neighbors_approx(q_new, rewire_radius)
        i_best, c_best = i_near, cost[i_near] + np.linalg.norm(q_new - nodes[i_near])
        for i in neigh:
            if collision_free(nodes[i], q_new):
                c = cost[i] + np.linalg.norm(q_new - nodes[i])
                if c < c_best:
                    c_best, i_best = c, i

        nodes.append(q_new)
        parent.append(i_best)
        cost.append(c_best)
        idx_new = len(nodes) - 1
        add_to_bins(idx_new, q_new)

        # rewire neighbors through q_new
        for i in neigh:
            if i == i_best: continue
            if collision_free(nodes[idx_new], nodes[i]):
                c = cost[idx_new] + np.linalg.norm(nodes[i] - nodes[idx_new])
                if c + 1e-6 < cost[i]:
                    parent[i] = idx_new
                    cost[i] = c

        # goal check
        if np.linalg.norm(q_new - goal) <= goal_tol and collision_free(q_new, goal):
            nodes.append(goal.copy())
            parent.append(idx_new)
            cost.append(cost[idx_new] + np.linalg.norm(goal - q_new))
            break

    # backtrack from the node closest to goal
    last = int(np.argmin(np.sum((np.asarray(nodes) - goal)**2, axis=1)))
    path = []
    while last != -1:
        path.append(tuple(map(int, np.round(nodes[last]))))
        last = parent[last]
    path.reverse()
    return path

def generate_ascii_maze(maze):
    rows, cols = len(maze), len(maze[0])
    out = []
    for r in range(rows):
        top = "".join("+" + ("---" if maze[r][c]['N'] or r == 0 else "   ") for c in range(cols)) + "+"
        mid = "".join(("|" if maze[r][c]['W'] or c == 0 else " ") + "   " for c in range(cols)) + "|"
        out += [top, mid]
    bottom = "+" + "+".join(["---"] * cols) + "+"   # <-- fixed
    out.append(bottom)
    return "\n".join(out)


# --- Pathfinding ---
DIRS = ['N', 'E', 'S', 'W']
delta = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
def left_dir(d):  return DIRS[(DIRS.index(d) + 1) % 4]   # CW
def right_dir(d): return DIRS[(DIRS.index(d) - 1) % 4]   # CCW
def opposite_dir(d):
    return {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[d]
def heuristic(r1, c1, r2, c2): return abs(r1 - r2) + abs(c1 - c2)

# ---------- Run RRT* only inside the center 5x5, stitch with grid path ----------
def hybrid_astar_plus_rrtstar(cropped_mask, rows, cols, cmds, start_full, roi_cells=5, inflate_px=4):
    h, w = cropped_mask.shape
    ch, cw = h / rows, w / cols

    # cells visited by grid A*
    cells = follow_cmds_to_cells(cmds, start_full)
    pts_grid = cells_to_pixel_centres(cells, h, w, rows, cols)

    # define the central ROI in cell indices
    r0 = (rows - roi_cells) // 2
    r1 = r0 + roi_cells - 1
    c0 = (cols - roi_cells) // 2
    c1 = c0 + roi_cells - 1

    # find first/last index of path inside ROI
    inside = [i for i, (r, c) in enumerate(cells) if r0 <= r <= r1 and c0 <= c <= c1]
    if len(inside) < 2:
        # A* never enters the ROI; just return the grid path
        return pts_grid

    i_enter, i_exit = inside[0], inside[-1]
    enter_px = pts_grid[i_enter]
    exit_px  = pts_grid[i_exit]

    # ROI pixel bounds
    y0 = int(round(r0 * ch)); y1 = int(round((r1 + 1) * ch))
    x0 = int(round(c0 * cw)); x1 = int(round((c1 + 1) * cw))

    roi = cropped_mask[y0:y1, x0:x1]

    # build a FREE-space mask with obstacle inflation (for robot radius/clearance)
    # obstacles = 1 where blocked
    obst = (roi == 0).astype(np.uint8) * 255
    if inflate_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*inflate_px+1, 2*inflate_px+1))
        obst = cv2.dilate(obst, k, iterations=1)
    free = cv2.bitwise_not(obst)  # 255 = free

    # start/goal in ROI coordinates
    sx, sy = enter_px[0] - x0, enter_px[1] - y0
    gx, gy = exit_px[0]  - x0, exit_px[1]  - y0

    path_roi = rrt_star(free, (sx, sy), (gx, gy))

    if len(path_roi) < 2:
        # fallback: keep grid path if RRT* fails
        return pts_grid

    # translate ROI path back to global pixels
    path_roi_global = [(x0 + x, y0 + y) for (x, y) in path_roi]

    # stitch: grid before enter, rrt* segment, grid after exit
    stitched = pts_grid[:i_enter+1] + path_roi_global + pts_grid[i_exit:]
    return stitched

# ---------- Draw any pixel polyline on top of the mask ----------
def overlay_pixel_polyline(cropped_mask, pts_px, color=(0,255,0), thickness=3):
    vis = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
    if len(pts_px) >= 2:
        arr = np.asarray(pts_px, dtype=np.int32)
        cv2.polylines(vis, [arr], isClosed=False, color=color,
                      thickness=thickness, lineType=cv2.LINE_AA)
        cv2.circle(vis, tuple(arr[0]),  max(3, thickness), (255,0,0), -1)  # start (blue)
        cv2.circle(vis, tuple(arr[-1]), max(3, thickness), (0,0,255), -1)  # end   (red)
    return vis


import math

def dir_to_deg(d):
    # Image coords: +x right, +y down.
    return {'E': 0.0, 'S': 90.0, 'W': 180.0, 'N': -90.0}[d.upper()]

def wrap180(a):
    # map to (-180, 180]
    a = (a + 180.0) % 360.0 - 180.0
    return a if a != -180.0 else 180.0

def segment_heading_deg(p, q):
    dx = q[0] - p[0]
    dy = q[1] - p[1]   # +y is down in images
    return math.degrees(math.atan2(dy, dx))

def segment_len(p, q):
    return math.hypot(q[0]-p[0], q[1]-p[1])

def polyline_to_commands(
    pts_px, img_h, img_w, rows, cols, start_dir='E',
    angle_tol_deg=5.0, round_angle_to=1,
    mode='repeat_f',            # 'repeat_f' or 'fN'
    cells_round_to=1,           # 1 -> integers; 10 -> 0.1 cells; 100 -> 0.01 cells
    numeric_prefix=False        # <<< NEW: if True and mode='fN', emit '0.3f' instead of 'f0.3'
):
    import math
    if len(pts_px) < 2:
        return ""

    cell_h = img_h / rows
    cell_w = img_w / cols
    px_per_cell = 0.5 * (cell_h + cell_w)

    cmds = []
    cur_heading = dir_to_deg(start_dir)

    i = 0
    while i+1 < len(pts_px) and segment_len(pts_px[i], pts_px[i+1]) < 1.0:
        i += 1
    if i+1 >= len(pts_px):
        return ""

    seg_head = segment_heading_deg(pts_px[i], pts_px[i+1])
    dtheta = wrap180(seg_head - cur_heading)
    if abs(dtheta) >= angle_tol_deg:
        turn = ('r', round(abs(dtheta)/round_angle_to)*round_angle_to) if dtheta > 0 else \
               ('l', round(abs(dtheta)/round_angle_to)*round_angle_to)
        cmds.append(turn)
        cur_heading = wrap180(cur_heading + (turn[1] if turn[0]=='l' else -turn[1]))

    run_px = 0.0
    cur_heading = seg_head
    for j in range(i, len(pts_px)-1):
        h = segment_heading_deg(pts_px[j], pts_px[j+1])
        L = segment_len(pts_px[j], pts_px[j+1])
        if L < 1.0:
            continue
        if abs(wrap180(h - cur_heading)) <= angle_tol_deg:
            run_px += L
        else:
            if run_px > 0:
                cells = run_px / px_per_cell
                if mode == 'repeat_f':
                    cmds.append(('f', int(round(cells))))
                else:
                    scale = cells_round_to
                    cmds.append(('F', round(cells*scale)/scale))
                run_px = 0.0
            dtheta = wrap180(h - cur_heading)
            tmag = round(abs(dtheta)/round_angle_to)*round_angle_to
            if tmag >= angle_tol_deg:
                cmds.append(('r', tmag) if dtheta > 0 else ('l', tmag))
            cur_heading = h
            run_px += L

    if run_px > 0:
        cells = run_px / px_per_cell
        if mode == 'repeat_f':
            cmds.append(('f', int(round(cells))))
        else:
            scale = cells_round_to
            cmds.append(('F', round(cells*scale)/scale))

    # --- Stringify ---
    if mode == 'repeat_f':
        out = []
        for t in cmds:
            if t[0] == 'f':
                out.append('f' * max(0, int(t[1])))
            else:
                out.append(f"{t[0]}{int(t[1])}")
        return "".join(out)
    else:
        # numeric distances in *cells*
        parts = []
        for t in cmds:
            if t[0] == 'F':
                v = t[1]
                v_str = f"{int(v)}" if abs(v - round(v)) < 1e-6 else f"{v}"
                parts.append(f"{v_str}f" if numeric_prefix else f"f{v_str}")  # <<< here
            else:
                parts.append(f"{t[0]}{int(t[1])}")
        return " ".join(parts)


# --- A* on (row, col, dir) state space -> ['l','r','f'] ---
DIRS = ['N', 'E', 'S', 'W']
delta = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
# flip sense of 'l' and 'r' everywhere
def left_dir(d):  return DIRS[(DIRS.index(d) + 1) % 4]
def right_dir(d): return DIRS[(DIRS.index(d) - 1) % 4]
def heuristic(r1, c1, r2, c2): return abs(r1 - r2) + abs(c1 - c2)

from queue import PriorityQueue

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
            return path  # list like ['f','f','r','f',...]

        # forward if no wall both sides of the edge
        dr, dc = delta[d]
        nr, nc = r + dr, c + dc
        if (0 <= nr < len(maze) and 0 <= nc < len(maze[0])
            and not maze[r][c][d]
            and not maze[nr][nc][opposite_dir(d)]):
            pq.put((g + 1 + heuristic(nr, nc, gr, gc), g + 1, nr, nc, d, path + ['f']))

        # in-place turns
        ld = left_dir(d)
        rd = right_dir(d)
        pq.put((g + 1 + heuristic(r, c, gr, gc), g + 1, r, c, ld, path + ['l']))
        pq.put((g + 1 + heuristic(r, c, gr, gc), g + 1, r, c, rd, path + ['r']))
    return []



def solve_maze_from_image(image_path, start, goal, start_dir='E', rows=9, cols=9):
    hsv_values = {'lower':[0,0,135], 'upper':[179,255,255]}
    mask, _ = load_and_mask_hsv(image_path, hsv_values)
    cleaned = clean_mask(mask)
    cropped = crop_to_outer_maze(cleaned, pad=5)
    maze = extract_maze_structure_centered(cropped, rows=rows, cols=cols)

    print("ASCII Maze:\n")
    print(generate_ascii_maze(maze))

    start_full = (start[0], start[1], start_dir.upper())
    cmds = solve_maze(maze, start_full, goal)

    # --- Hybrid: RRT* only inside the central 5x5 ---
    all_pts_px = hybrid_astar_plus_rrtstar(
        cropped_mask=cropped, rows=rows, cols=cols,
        cmds=cmds, start_full=start_full,
        roi_cells=5, inflate_px=5
    )

    # Build the compact motion string with turn angles
    h, w = cropped.shape
    motion_str = polyline_to_commands(
    pts_px=all_pts_px, img_h=h, img_w=w, rows=rows, cols=cols,
    start_dir=start_dir,
    angle_tol_deg=7, round_angle_to=1,
    mode='fN',            # numeric distances
    cells_round_to=10,    # 0.1-cell resolution
    numeric_prefix= False   # -> '0.4f r45 1.2f l90 ...'
    )
    print(motion_str)
    overlay = overlay_pixel_polyline(
        cropped_mask=cropped, pts_px=all_pts_px,
        thickness=max(2, int(min(h/rows, w/cols)//5))
    )

    plt.figure(figsize=(6,6))
    plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
    plt.title("Hybrid A* + RRT* (RRT* only in center 5×5)")
    plt.axis('off')
    plt.show()
