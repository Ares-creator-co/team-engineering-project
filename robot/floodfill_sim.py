# Laptop simulator for floodfill.py. Runs on normal Python 3, NOT on the Pico.
#
#   python3 floodfill_sim.py                 one random 16x16 maze, drawn with the route
#   python3 floodfill_sim.py --seed 7        a specific maze (same seed = same maze)
#   python3 floodfill_sim.py --maze my.txt   a maze drawn in a text file (see format_maze)
#   python3 floodfill_sim.py --batch 200     test on 200 mazes and print averages
#
# The times are ESTIMATES from the constants below. Replace them with timings
# measured on the real robot once it can drive a cell and turn.
import argparse
import random

from floodfill import (N, E, S, W, DIRS, DX, DY, Maze, Explorer, goal_cells,
                       turn_amount, STRAIGHT, BACK)

# --- time model (seconds), guesses until measured on the robot ---
EXPLORE_CELL_S = 1.0   # drive one cell, stop, read the sensors
SPEED_CELL_S = 0.5     # one cell during the speed run (~0.36 m/s)
TURN_S = 0.6           # 90 degree turn
U_TURN_S = 1.0         # 180 degree turn
TIME_TO_BEAT_S = 87    # 1:27


# ---------- maze making ----------

def remove_wall(maze, x, y, d):
    maze.set_wall(x, y, d, wall=False)


def post_walls(maze, px, py):
    # number of walls touching lattice post (px, py); posts run 0..size in each direction
    size = maze.size

    def horizontal(cx, line):      # wall along y = line, under cell column cx
        return maze.has_wall(cx, line, S) if line < size else maze.has_wall(cx, size - 1, N)

    def vertical(line, cy):        # wall along x = line, beside cell row cy
        return maze.has_wall(line, cy, W) if line < size else maze.has_wall(size - 1, cy, E)

    count = 0
    if px > 0:
        count += horizontal(px - 1, py)
    if px < size:
        count += horizontal(px, py)
    if py > 0:
        count += vertical(px, py - 1)
    if py < size:
        count += vertical(px, py)
    return count


def generate_maze(size=16, seed=None, loops=0.08):
    # competition-style maze: start in the corner walled on 3 sides, 2x2 goal in the
    # middle with one entrance, every post touches a wall, and some loops so there
    # is more than one route
    rng = random.Random(seed)
    maze = Maze(size)
    for y in range(size):
        for x in range(size):
            for d in DIRS:
                maze.set_wall(x, y, d)

    goals = set(goal_cells(size))
    start = (0, 0)

    # carve a perfect maze over every cell except the start and the goal square
    visited = set(goals) | {start}
    stack = [(0, 1)]
    visited.add((0, 1))
    while stack:
        x, y = stack[-1]
        options = [d for d in DIRS
                   if maze.inside(x + DX[d], y + DY[d]) and (x + DX[d], y + DY[d]) not in visited]
        if not options:
            stack.pop()
            continue
        d = rng.choice(options)
        remove_wall(maze, x, y, d)
        nxt = (x + DX[d], y + DY[d])
        visited.add(nxt)
        stack.append(nxt)

    remove_wall(maze, 0, 0, N)   # the start's only exit

    # open the goal square up inside, then give it exactly one entrance
    (ax, ay), (bx, _), _, (_, by) = goal_cells(size)
    remove_wall(maze, ax, ay, E)
    remove_wall(maze, ax, by, E)
    remove_wall(maze, ax, ay, N)
    remove_wall(maze, bx, ay, N)
    entrances = []
    for gx, gy in goals:
        for d in DIRS:
            if (gx + DX[d], gy + DY[d]) not in goals:
                entrances.append((gx, gy, d))
    remove_wall(maze, *rng.choice(entrances))

    # knock out a few extra walls for loops, keeping the rules above
    for y in range(size):
        for x in range(size):
            for d in (N, E):
                nx, ny = x + DX[d], y + DY[d]
                if not maze.inside(nx, ny) or not maze.has_wall(x, y, d):
                    continue
                if {(x, y), (nx, ny)} & (goals | {start}):
                    continue
                if rng.random() > loops:
                    continue
                if d == E:
                    posts = ((x + 1, y), (x + 1, y + 1))
                else:
                    posts = ((x, y + 1), (x + 1, y + 1))
                if all(post_walls(maze, px, py) >= 2 for px, py in posts):
                    remove_wall(maze, x, y, d)

    # the simulator knows every wall of the real maze
    maze.known = bytearray([0xF] * (size * size))
    return maze


# ---------- text format ----------
# +---+---+       top line is the north edge
# |       |       '|' is a wall between cells, '---' is a wall above/below
# +   +---+
# |   |   |
# +---+---+

def format_maze(maze, marks=None):
    marks = marks or {}
    size = maze.size
    lines = []
    for y in range(size - 1, -1, -1):
        top = "+"
        row = ""
        for x in range(size):
            top += ("---" if maze.has_wall(x, y, N) else "   ") + "+"
            row += ("|" if maze.has_wall(x, y, W) else " ") + " %s " % marks.get((x, y), " ")
        row += "|" if maze.has_wall(size - 1, y, E) else " "
        lines.append(top)
        lines.append(row)
    lines.append("+" + "---+" * size)
    return "\n".join(lines)


def parse_maze(text):
    rows = [line.rstrip("\n") for line in text.strip("\n").splitlines()]
    size = (len(rows) - 1) // 2
    maze = Maze(size)
    for r in range(size):
        y = size - 1 - r
        top, mid, bottom = rows[2 * r], rows[2 * r + 1].ljust(4 * size + 1), rows[2 * r + 2]
        for x in range(size):
            maze.set_wall(x, y, N, top[4 * x + 2] == "-")
            maze.set_wall(x, y, S, bottom[4 * x + 2] == "-")
            maze.set_wall(x, y, W, mid[4 * x] == "|")
            maze.set_wall(x, y, E, mid[4 * x + 4] == "|")
    return maze


# ---------- simulated robot ----------

class SimRobot:
    def __init__(self, real_maze):
        self.real = real_maze
        self.cells = 0
        self.turns = 0
        self.u_turns = 0
        self.heading = N
        self.visited = {(0, 0)}
        self.pos = (0, 0)

    def sense(self, x, y, heading):
        # left, front and right sensors; at the very start the back wall is known too
        seen = {}
        for amount in (3, 0, 1):
            d = DIRS[(DIRS.index(heading) + amount) % 4]
            seen[d] = self.real.has_wall(x, y, d)
        if (x, y) == (0, 0):
            seen[S] = True
        return seen

    def move(self, d):
        if self.real.has_wall(self.pos[0], self.pos[1], d):
            raise RuntimeError("robot drove into a wall at %s" % (self.pos,))
        amount = turn_amount(self.heading, d)
        if amount == BACK:
            self.u_turns += 1
        elif amount != STRAIGHT:
            self.turns += 1
        self.heading = d
        self.pos = (self.pos[0] + DX[d], self.pos[1] + DY[d])
        self.visited.add(self.pos)
        self.cells += 1

    def explore_time(self):
        return self.cells * EXPLORE_CELL_S + self.turns * TURN_S + self.u_turns * U_TURN_S


def speed_run_time(path):
    turns = sum(1 for a, b in zip(path, path[1:]) if a != b)
    return len(path) * SPEED_CELL_S + turns * TURN_S


def run(real_maze):
    # explores the maze from scratch, then plans the speed run. Returns the results.
    size = real_maze.size
    goals = goal_cells(size)
    robot = SimRobot(real_maze)
    explorer = Explorer(Maze(size), robot.sense, robot.move)

    first_cells = explorer.go_to(goals)
    first_time = robot.explore_time()
    explorer.go_to([(0, 0)])
    trips = 1
    while not explorer.maze.route_proven((0, 0), goals) and trips < 4:
        explorer.go_to(goals)
        explorer.go_to([(0, 0)])
        trips += 1

    path = explorer.maze.plan((0, 0), N, goals, unknown_is_wall=True)
    true_best = Maze.plan(real_maze, (0, 0), N, goals, unknown_is_wall=False)
    return {
        "robot": robot,
        "explorer": explorer,
        "first_cells": first_cells,
        "first_time": first_time,
        "trips": trips,
        "explore_cells": robot.cells,
        "explore_time": robot.explore_time(),
        "path": path,
        "speed_time": speed_run_time(path),
        "optimal": len(path) == len(true_best),
        "true_best_cells": len(true_best),
    }


def show(real_maze):
    result = run(real_maze)
    marks = {cell: "." for cell in result["robot"].visited}
    x, y = 0, 0
    for d in result["path"]:
        x, y = x + DX[d], y + DY[d]
        marks[(x, y)] = "*"
    for cell in goal_cells(real_maze.size):
        marks[cell] = "G"
    marks[(0, 0)] = "S"
    print(format_maze(real_maze, marks))
    print("S start   G goal   . cells visited while exploring   * speed run route\n")
    print("First run to the goal:  %3d cells, about %5.1f s %s"
          % (result["first_cells"], result["first_time"],
             "(beats 1:27)" if result["first_time"] < TIME_TO_BEAT_S else "(slower than 1:27)"))
    print("Exploring in total:     %3d cells over %d round trip(s), about %.0f s"
          % (result["explore_cells"], result["trips"], result["explore_time"]))
    print("Speed run:              %3d cells, about %5.1f s (shortest possible: %d cells%s)"
          % (len(result["path"]), result["speed_time"], result["true_best_cells"],
             ", found it" if result["optimal"] else ", NOT found"))


def batch(count, size):
    results = [run(generate_maze(size, seed)) for seed in range(count)]
    n = len(results)

    def avg(key):
        return sum(r[key] for r in results) / n

    print("%d random %dx%d mazes (times use the guessed constants at the top of the file)\n" % (n, size, size))
    print("First run to goal:  avg %.0f cells, avg %.0f s, beats 1:27 in %d/%d"
          % (avg("first_cells"), avg("first_time"),
             sum(r["first_time"] < TIME_TO_BEAT_S for r in results), n))
    print("Exploring overall:  avg %.0f cells, avg %.1f round trips"
          % (avg("explore_cells"), avg("trips")))
    print("Speed run:          avg %.0f cells, avg %.0f s, beats 1:27 in %d/%d"
          % (sum(len(r["path"]) for r in results) / n, avg("speed_time"),
             sum(r["speed_time"] < TIME_TO_BEAT_S for r in results), n))
    print("Speed run is the true shortest route in %d/%d mazes" % (sum(r["optimal"] for r in results), n))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flood fill micromouse simulator")
    parser.add_argument("--seed", type=int, help="maze number to generate")
    parser.add_argument("--maze", help="text file with a maze drawing")
    parser.add_argument("--batch", type=int, help="test this many random mazes")
    parser.add_argument("--size", type=int, default=16)
    args = parser.parse_args()

    if args.batch:
        batch(args.batch, args.size)
    elif args.maze:
        with open(args.maze) as f:
            show(parse_maze(f.read()))
    else:
        seed = args.seed if args.seed is not None else random.randrange(10000)
        print("maze seed:", seed)
        show(generate_maze(args.size, seed))
