import heapq
import itertools
import tkinter as tk
import time
import random  # Rastgelelik için eklendi

# ========== GRID SETTINGS ==========
ROWS = 15
COLS = 20
CELL_SIZE = 40  # pixels
MAX_CANVAS_WIDTH = 900
MAX_CANVAS_HEIGHT = 700

# 0 = empty, 1 = obstacle
grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

start = None          # red dot (single)
goals = set()         # multiple green dots

current_mode = "start"       # "start", "goal", "wall"
current_algorithm = "astar"  # "astar" or "dijkstra"

# Tkinter setup
root = tk.Tk()
root.title("A* vs Dijkstra - Multi-Goal Shortest Tour")

width = COLS * CELL_SIZE
height = ROWS * CELL_SIZE

canvas = tk.Canvas(root, width=width, height=height, bg="white")
canvas.pack()

rects = [[None for _ in range(COLS)] for _ in range(ROWS)]
npc_id = None


# ========== ALGORITHM HELPER FUNCTIONS ==========

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbors(pos):
    r, c = pos
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] == 0:
            yield (nr, nc)


def reconstruct_path(came_from, start_node, goal_node):
    if goal_node not in came_from and goal_node != start_node:
        return None
    path = [goal_node]
    cur = goal_node
    while cur != start_node:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path

def search_with_steps(start_node, goal_node, algorithm):
    INF = float('inf')
    use_heuristic = (algorithm == "astar")

    g = {start_node: 0}
    came_from = {}
    start_h = manhattan(start_node, goal_node) if use_heuristic else 0
    f = {start_node: start_h}
    open_heap = [(f[start_node], start_node)]
    closed = set()
    steps = []

    t0 = time.perf_counter()

    while open_heap:
        cur_f, node = heapq.heappop(open_heap)
        if node in closed:
            continue
        closed.add(node)

        open_nodes = [n for (_, n) in open_heap]
        steps.append({
            "current": node,
            "closed": set(closed),
            "open": set(open_nodes),
        })

        if node == goal_node:
            path = reconstruct_path(came_from, start_node, goal_node)
            t1 = time.perf_counter()
            return path, len(closed), steps, (t1 - t0) * 1000.0  # ms

        for nb in neighbors(node):
            tentative_g = g[node] + 1
            if tentative_g < g.get(nb, INF):
                came_from[nb] = node
                g[nb] = tentative_g
                h = manhattan(nb, goal_node) if use_heuristic else 0
                f_nb = tentative_g + h
                f[nb] = f_nb
                heapq.heappush(open_heap, (f_nb, nb))

    t1 = time.perf_counter()
    return None, len(closed), steps, (t1 - t0) * 1000.0

def search_simple(start_node, goal_node, algorithm):
    path, visited, steps, t_ms = search_with_steps(start_node, goal_node, algorithm)
    return path, visited, t_ms


def compute_best_order(start_node, goals_set):
    if not goals_set:
        return None, None, None

    points = [start_node] + list(goals_set)
    n = len(points)
    INF = float('inf')

    dist = [[INF] * n for _ in range(n)]
    for i in range(n):
        dist[i][i] = 0

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            p_i = points[i]
            p_j = points[j]
            path_ij, _, _ = search_simple(p_i, p_j, "dijkstra")
            if path_ij is None:
                dist[i][j] = INF
            else:
                dist[i][j] = len(path_ij) - 1

    goal_indices = list(range(1, n))
    best_order = None
    best_cost = INF

    for perm in itertools.permutations(goal_indices):
        cost = 0
        prev = 0  # start
        feasible = True
        for idx in perm:
            if dist[prev][idx] == INF:
                feasible = False
                break
            cost += dist[prev][idx]
            prev = idx
        if not feasible:
            continue
        if cost < best_cost:
            best_cost = cost
            best_order = perm

    if best_order is None:
        return None, None, None

    return points, best_order, best_cost


# ========== DRAWING / INTERFACE HELPER FUNCTIONS ==========

def init_grid_draw():
    global npc_id, rects
    canvas.delete("all")
    rects = [[None for _ in range(COLS)] for _ in range(ROWS)]

    for r in range(ROWS):
        for c in range(COLS):
            x1 = c * CELL_SIZE
            y1 = r * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE
            rect_id = canvas.create_rectangle(
                x1, y1, x2, y2,
                fill="white", outline="gray"
            )
            rects[r][c] = rect_id

    npc_id = canvas.create_oval(-10, -10, -5, -5, fill="red", outline="red")


def update_mode_label():
    mode_label.config(
        text=f"Mode: {current_mode.upper()}  |  Alg: {current_algorithm.upper()}  |  "
             f"S:Start  G:Goal  W:Wall  A:A* D:Dijkstra  C:Random  Space:Run  R:Reset"
    )


def redraw_static():
    # Update square colors
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 1:
                color = "black"
            else:
                color = "white"
            canvas.itemconfig(rects[r][c], fill=color)

    # Goals (green)
    for (r, c) in goals:
        x1 = c * CELL_SIZE + CELL_SIZE * 0.25
        y1 = r * CELL_SIZE + CELL_SIZE * 0.25
        x2 = c * CELL_SIZE + CELL_SIZE * 0.75
        y2 = r * CELL_SIZE + CELL_SIZE * 0.75
        canvas.create_oval(x1, y1, x2, y2, fill="green", outline="green")

    # Start (red)
    if start is not None:
        r, c = start
        x1 = c * CELL_SIZE + CELL_SIZE * 0.25
        y1 = r * CELL_SIZE + CELL_SIZE * 0.25
        x2 = c * CELL_SIZE + CELL_SIZE * 0.75
        y2 = r * CELL_SIZE + CELL_SIZE * 0.75
        canvas.coords(npc_id, x1, y1, x2, y2)
    else:
        canvas.coords(npc_id, -10, -10, -5, -5)


def reset_all():
    global grid, start, goals
    grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    start = None
    goals.clear()
    init_grid_draw()
    redraw_static()
    info_label.config(
        text="All reset. S:Start, G:Goal, W:Wall, C:Random, Space:Run."
    )


def generate_random_scenario():
    global grid, start, goals

    # 1. Reset everything first
    grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    start = None
    goals.clear()

    # 2. Place random walls (25% probability)
    for r in range(ROWS):
        for c in range(COLS):
            if random.random() < 0.25:
                grid[r][c] = 1

    # Helper to find a free cell
    def get_random_free_cell():
        # Tries 1000 times to find a free cell to avoid infinite loops in full maps
        for _ in range(1000):
            r = random.randint(0, ROWS - 1)
            c = random.randint(0, COLS - 1)
            if grid[r][c] == 0:
                return (r, c)
        return None

    # 3. Place Start
    s_pos = get_random_free_cell()
    if s_pos:
        start = s_pos
    else:
        # If map is full of walls, just reset and return
        reset_all()
        info_label.config(text="Could not place Start (Map too dense). Try again.")
        return

    # 4. Place Goals (1 to 5)
    num_goals = random.randint(1, 5)
    for _ in range(num_goals):
        g_pos = get_random_free_cell()
        # Ensure distinct goal (not start, not existing goal)
        if g_pos and g_pos != start and g_pos not in goals:
            goals.add(g_pos)

    # 5. Redraw
    init_grid_draw()
    redraw_static()
    info_label.config(text=f"Random scenario: {len(goals)} goals created. Select Algo and press Space.")


def apply_new_grid_size():
    global ROWS, COLS, CELL_SIZE, grid, start, goals, width, height

    try:
        new_rows = int(row_entry.get())
        new_cols = int(col_entry.get())
    except:
        info_label.config(text="Enter a valid number for Rows and Cols!")
        return

    if new_rows < 3 or new_cols < 3:
        info_label.config(text="Map must be at least 3x3.")
        return

    ROWS = new_rows
    COLS = new_cols

    cell_w = MAX_CANVAS_WIDTH // COLS
    cell_h = MAX_CANVAS_HEIGHT // ROWS
    new_cell_size = min(cell_w, cell_h)
    if new_cell_size < 10:
        new_cell_size = 10
    if new_cell_size > 60:
        new_cell_size = 60

    CELL_SIZE = new_cell_size

    grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    start = None
    goals.clear()

    width = COLS * CELL_SIZE
    height = ROWS * CELL_SIZE
    canvas.config(width=width, height=height)

    init_grid_draw()
    redraw_static()

    info_label.config(
        text=f"Map updated to {ROWS} x {COLS}. Cell: {CELL_SIZE}px"
    )
    canvas.focus_set()


def cell_from_xy(x, y):
    c = x // CELL_SIZE
    r = y // CELL_SIZE
    if 0 <= r < ROWS and 0 <= c < COLS:
        return (r, c)
    return None


def on_click(event):
    global start, goals

    cell = cell_from_xy(event.x, event.y)
    if cell is None:
        return

    r, c = cell

    if current_mode == "start":
        if grid[r][c] == 1:
            grid[r][c] = 0
        start = (r, c)

    elif current_mode == "goal":
        if grid[r][c] == 1:
            grid[r][c] = 0
        if (r, c) in goals:
            goals.remove((r, c))
        else:
            if start is not None and (r, c) == start:
                return
            goals.add((r, c))

    elif current_mode == "wall":
        if start is not None and (r, c) == start:
            return
        if (r, c) in goals:
            return
        grid[r][c] = 1 - grid[r][c]

    init_grid_draw()
    redraw_static()


def on_key(event):
    global current_mode, current_algorithm

    ch = event.char.lower()

    if ch == 's':
        current_mode = "start"
        update_mode_label()
        canvas.focus_set()
        return "break"

    elif ch == 'g':
        current_mode = "goal"
        update_mode_label()
        canvas.focus_set()
        return "break"

    elif ch == 'w':
        current_mode = "wall"
        update_mode_label()
        canvas.focus_set()
        return "break"

    elif ch == 'a':
        current_algorithm = "astar"
        update_mode_label()
        info_label.config(text="Algorithm: A* selected.")
        canvas.focus_set()
        return "break"

    elif ch == 'd':
        current_algorithm = "dijkstra"
        update_mode_label()
        info_label.config(text="Algorithm: Dijkstra selected.")
        canvas.focus_set()
        return "break"

    elif ch == 'r':
        reset_all()
        update_mode_label()
        canvas.focus_set()
        return "break"
    
    elif ch == 'c':  # NEW: Create Random Scenario
        generate_random_scenario()
        update_mode_label()
        canvas.focus_set()
        return "break"

    elif event.keysym == "space":
        run_tour()
        canvas.focus_set()
        return "break"


def animate_full_tour(full_path, idx, on_finish):
    if not full_path or idx >= len(full_path):
        on_finish()
        return

    r, c = full_path[idx]
    if grid[r][c] == 0:
        canvas.itemconfig(rects[r][c], fill="#00dddd")

    x1 = c * CELL_SIZE + CELL_SIZE * 0.25
    y1 = r * CELL_SIZE + CELL_SIZE * 0.25
    x2 = c * CELL_SIZE + CELL_SIZE * 0.75
    y2 = r * CELL_SIZE + CELL_SIZE * 0.75
    canvas.coords(npc_id, x1, y1, x2, y2)

    root.after(150, lambda: animate_full_tour(full_path, idx + 1, on_finish))


def animate_segment_tour(seq_points, seg_idx, total_segments,
                         chosen_alg, chosen_metrics, other_alg_metrics,
                         full_path_segments, best_cost,
                         full_tour_path, comparison_text):

    if seg_idx >= len(seq_points) - 1:
        # 1) Clear grid (walls fixed, colors reset)
        for r in range(ROWS):
            for c in range(COLS):
                if grid[r][c] == 1:
                    color = "black"
                else:
                    color = "white"
                canvas.itemconfig(rects[r][c], fill=color)

        redraw_static()

        # 2) Place NPC at start
        if start is not None:
            r0, c0 = start
            x1 = c0 * CELL_SIZE + CELL_SIZE * 0.25
            y1 = r0 * CELL_SIZE + CELL_SIZE * 0.25
            x2 = c0 * CELL_SIZE + CELL_SIZE * 0.75
            y2 = r0 * CELL_SIZE + CELL_SIZE * 0.75
            canvas.coords(npc_id, x1, y1, x2, y2)

        info_label.config(
            text=f"Shortest tour found with {chosen_alg.upper()} is re-running from start..."
        )

        # 3) Execute full tour from start to finish
        def after_full_tour():
            info_label.config(text=comparison_text)

        animate_full_tour(full_tour_path, 0, after_full_tour)
        return

    seg_start = seq_points[seg_idx]
    seg_goal = seq_points[seg_idx + 1]

    path, visited, steps, t_ms = full_path_segments[seg_idx]

    if path is None:
        info_label.config(
            text=f"Path not found for segment {seg_idx+1} with {chosen_alg.upper()}. Check obstacles/positions."
        )
        return

    def animate_search(i):
        if i >= len(steps):
            animate_path(0)
            return

        step = steps[i]
        current = step["current"]
        closed = step["closed"]
        open_set = step["open"]

        for r in range(ROWS):
            for c in range(COLS):
                if grid[r][c] == 1:
                    color = "black"
                else:
                    color = "white"
                canvas.itemconfig(rects[r][c], fill=color)

        for (r, c) in closed:
            if grid[r][c] == 0:
                canvas.itemconfig(rects[r][c], fill="#d0d0d0")

        for (r, c) in open_set:
            if grid[r][c] == 0:
                canvas.itemconfig(rects[r][c], fill="#ffffaa")

        r, c = current
        if grid[r][c] == 0:
            canvas.itemconfig(rects[r][c], fill="#88ccff")

        x1 = c * CELL_SIZE + CELL_SIZE * 0.25
        y1 = r * CELL_SIZE + CELL_SIZE * 0.25
        x2 = c * CELL_SIZE + CELL_SIZE * 0.75
        y2 = r * CELL_SIZE + CELL_SIZE * 0.75
        canvas.coords(npc_id, x1, y1, x2, y2)

        info_label.config(
            text=f"{chosen_alg.upper()} – Segment {seg_idx+1}/{total_segments}: {seg_start} → {seg_goal} | Search steps..."
        )

        root.after(80, lambda: animate_search(i + 1))

    def animate_path(i):
        if i >= len(path):
            root.after(300, lambda: animate_segment_tour(
                seq_points, seg_idx + 1, total_segments,
                chosen_alg, chosen_metrics, other_alg_metrics,
                full_path_segments, best_cost,
                full_tour_path, comparison_text
            ))
            return

        r, c = path[i]
        if grid[r][c] == 0:
            canvas.itemconfig(rects[r][c], fill="#00dddd")

        x1 = c * CELL_SIZE + CELL_SIZE * 0.25
        y1 = r * CELL_SIZE + CELL_SIZE * 0.25
        x2 = c * CELL_SIZE + CELL_SIZE * 0.75
        y2 = r * CELL_SIZE + CELL_SIZE * 0.75
        canvas.coords(npc_id, x1, y1, x2, y2)

        root.after(150, lambda: animate_path(i + 1))

    animate_search(0)


def run_tour():
    if start is None:
        info_label.config(text="First place the red start point. (Select mode with S)")
        return
    if not goals:
        info_label.config(text="Place at least one green goal point. (Select mode with G)")
        return

    points, best_order, best_cost = compute_best_order(start, goals)
    if best_order is None:
        info_label.config(
            text="A complete tour reaching all targets could not be found (some targets unreachable)."
        )
        return

    seq_points = [points[0]] + [points[i] for i in best_order]
    total_segments = len(seq_points) - 1

    chosen_alg = current_algorithm
    other_alg = "astar" if chosen_alg == "dijkstra" else "dijkstra"

    chosen_total_path_len = 0
    chosen_total_visited = 0
    chosen_total_time = 0.0

    other_total_path_len = 0
    other_total_visited = 0
    other_total_time = 0.0

    chosen_segments_data = []

    for i in range(total_segments):
        s = seq_points[i]
        g = seq_points[i + 1]

        path_c, visited_c, steps_c, t_ms_c = search_with_steps(s, g, chosen_alg)
        if path_c is None:
            info_label.config(
                text=f"Path not found for segment {i+1} with {chosen_alg.upper()}. Check obstacles/positions."
            )
            return

        chosen_segments_data.append((path_c, visited_c, steps_c, t_ms_c))
        chosen_total_path_len += (len(path_c) - 1)
        chosen_total_visited += visited_c
        chosen_total_time += t_ms_c

        path_o, visited_o, t_ms_o = search_simple(s, g, other_alg)
        if path_o is None:
            info_label.config(
                text=f"Path not found for segment {i+1} with {other_alg.upper()}. Check obstacles/positions."
            )
            return
        other_total_path_len += (len(path_o) - 1)
        other_total_visited += visited_o
        other_total_time += t_ms_o

    # Full tour path (combine paths found by the chosen algorithm)
    full_tour_path = []
    for idx, (p, v, st, tm) in enumerate(chosen_segments_data):
        if idx == 0:
            full_tour_path.extend(p)
        else:
            full_tour_path.extend(p[1:])  # do not repeat the common node with the previous goal

    chosen_metrics = (chosen_total_path_len, chosen_total_visited, chosen_total_time)
    other_metrics = (other_total_path_len, other_total_visited, other_total_time)

    comparison_text = (
        f"Total theoretical shortest tour length (edges): {best_cost}  |  "
        f"{chosen_alg.upper()} → path={chosen_total_path_len}, visited={chosen_total_visited}, time={chosen_total_time:.2f} ms  |  "
        f"{('A*' if chosen_alg=='dijkstra' else 'Dijkstra')} → path={other_total_path_len}, "
        f"visited={other_total_visited}, time={other_total_time:.2f} ms"
    )

    # Phase 1: segment by segment display
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 1:
                color = "black"
            else:
                color = "white"
            canvas.itemconfig(rects[r][c], fill=color)

    redraw_static()

    info_label.config(
        text=f"Starting shortest tour animation to all goals with {chosen_alg.upper()}..."
    )

    animate_segment_tour(
        seq_points, 0, total_segments,
        chosen_alg, chosen_metrics, other_metrics,
        chosen_segments_data, best_cost,
        full_tour_path, comparison_text
    )


# ========== GRID SIZE PANEL AND LABELS ==========

size_frame = tk.Frame(root)
size_frame.pack(pady=5)

tk.Label(size_frame, text="Rows: ").pack(side=tk.LEFT)
row_entry = tk.Entry(size_frame, width=5)
row_entry.insert(0, str(ROWS))
row_entry.pack(side=tk.LEFT)

tk.Label(size_frame, text="Cols: ").pack(side=tk.LEFT)
col_entry = tk.Entry(size_frame, width=5)
col_entry.insert(0, str(COLS))
col_entry.pack(side=tk.LEFT)

apply_button = tk.Button(size_frame, text="Apply", command=apply_new_grid_size)
apply_button.pack(side=tk.LEFT, padx=10)

info_label = tk.Label(root, text="", font=("Arial", 12))
info_label.pack(pady=5)

mode_label = tk.Label(root, text="", font=("Arial", 12, "bold"))
mode_label.pack(pady=5)

# Initial drawing and event binding
init_grid_draw()
redraw_static()
update_mode_label()

canvas.bind("<Button-1>", on_click)
root.bind("<Key>", on_key)

info_label.config(
    text="S: Start, G: Goal, W: Wall, C: Random Scenario, A: A*, D: Dijkstra, Space: Run, R: Reset."
)

root.mainloop()