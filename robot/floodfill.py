# Flood fill maze solver for the micromouse.
# Plain Python: the same file runs on the laptop (floodfill_sim.py) and on the Pico.
#
# The maze is a grid of cells. Walls sit BETWEEN cells, so each cell stores which
# of its 4 sides has a wall, as bits: N=1, E=2, S=4, W=8.
# x goes east (0 = left), y goes north (0 = bottom). The robot starts at (0, 0) facing north.

N, E, S, W = 1, 2, 4, 8
DIRS = (N, E, S, W)
DX = {N: 0, E: 1, S: 0, W: -1}
DY = {N: 1, E: 0, S: -1, W: 0}
OPPOSITE = {N: S, E: W, S: N, W: E}
NAMES = {N: "N", E: "E", S: "S", W: "W"}
UNREACHABLE = 9999

STRAIGHT, RIGHT, BACK, LEFT = 0, 1, 2, 3


def turn_amount(heading, direction):
    # how far the robot turns to face `direction`: STRAIGHT, RIGHT, BACK or LEFT
    return (DIRS.index(direction) - DIRS.index(heading)) % 4


def rotate(heading, amount):
    return DIRS[(DIRS.index(heading) + amount) % 4]


def goal_cells(size):
    # the 2x2 square in the middle of the maze
    a, b = size // 2 - 1, size // 2
    return [(a, a), (b, a), (a, b), (b, b)]


class Maze:
    def __init__(self, size=16):
        self.size = size
        self.walls = bytearray(size * size)   # wall bits for each cell
        self.known = bytearray(size * size)   # which sides have been seen
        # the outside walls are always there
        for i in range(size):
            self.set_wall(i, 0, S)
            self.set_wall(i, size - 1, N)
            self.set_wall(0, i, W)
            self.set_wall(size - 1, i, E)

    def inside(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    def has_wall(self, x, y, d):
        return bool(self.walls[y * self.size + x] & d)

    def is_known(self, x, y, d):
        return bool(self.known[y * self.size + x] & d)

    def _mark(self, x, y, d, wall):
        i = y * self.size + x
        self.known[i] |= d
        if wall:
            self.walls[i] |= d
        else:
            self.walls[i] &= 0xF ^ d

    def set_wall(self, x, y, d, wall=True):
        # records a wall (or no wall) on side d of (x, y), and the matching side of the neighbour
        self._mark(x, y, d, wall)
        nx, ny = x + DX[d], y + DY[d]
        if self.inside(nx, ny):
            self._mark(nx, ny, OPPOSITE[d], wall)

    def blocked(self, x, y, d, unknown_is_wall=False):
        if self.has_wall(x, y, d):
            return True
        return unknown_is_wall and not self.is_known(x, y, d)

    def flood(self, goals, unknown_is_wall=False):
        # distance in cells from every cell to the nearest goal cell.
        # unknown_is_wall=False: assume unseen walls are open (for exploring).
        # unknown_is_wall=True: only trust what has been seen (for the speed run).
        size = self.size
        dist = [UNREACHABLE] * (size * size)
        queue = []
        for x, y in goals:
            dist[y * size + x] = 0
            queue.append((x, y))
        head = 0
        while head < len(queue):
            x, y = queue[head]
            head += 1
            next_dist = dist[y * size + x] + 1
            for d in DIRS:
                if self.blocked(x, y, d, unknown_is_wall):
                    continue
                i = (y + DY[d]) * size + x + DX[d]
                if dist[i] > next_dist:
                    dist[i] = next_dist
                    queue.append((x + DX[d], y + DY[d]))
        return dist

    def distance(self, dist, x, y):
        return dist[y * self.size + x]

    def best_direction(self, dist, x, y, heading, unknown_is_wall=False):
        # the open neighbour closest to the goal; prefers going straight to save turning
        best = None
        best_dist = UNREACHABLE
        for amount in (STRAIGHT, RIGHT, LEFT, BACK):
            d = rotate(heading, amount)
            if self.blocked(x, y, d, unknown_is_wall):
                continue
            nd = dist[(y + DY[d]) * self.size + x + DX[d]]
            if nd < best_dist:
                best, best_dist = d, nd
        return best

    def plan(self, start, heading, goals, unknown_is_wall=True):
        # list of directions from start to the goal, or None if there is no known route
        dist = self.flood(goals, unknown_is_wall)
        x, y = start
        if self.distance(dist, x, y) == UNREACHABLE:
            return None
        path = []
        while self.distance(dist, x, y) != 0:
            d = self.best_direction(dist, x, y, heading, unknown_is_wall)
            path.append(d)
            x, y, heading = x + DX[d], y + DY[d], d
        return path

    def route_proven(self, start, goals):
        # True once the shortest route through seen cells can't be beaten by any unseen gap
        optimistic = self.distance(self.flood(goals, False), *start)
        pessimistic = self.distance(self.flood(goals, True), *start)
        return optimistic == pessimistic


class Explorer:
    # Drives the robot through the maze with flood fill.
    #   sense(x, y, heading) -> {direction: True if wall, False if open} for every side it can see
    #   move(direction)      -> turns to face `direction` and drives exactly one cell
    # On the laptop these are simulated; on the Pico they use the ToF sensors and motors.
    def __init__(self, maze, sense, move, start=(0, 0), heading=N):
        self.maze = maze
        self.sense = sense
        self.move = move
        self.start = start
        self.x, self.y = start
        self.heading = heading

    def look(self):
        for d, wall in self.sense(self.x, self.y, self.heading).items():
            self.maze.set_wall(self.x, self.y, d, wall)

    def go_to(self, goals, max_moves=2000):
        # explores towards the goals, re-flooding after every cell. Returns the moves made.
        moves = 0
        self.look()
        while (self.x, self.y) not in goals:
            dist = self.maze.flood(goals)
            if self.maze.distance(dist, self.x, self.y) == UNREACHABLE:
                raise RuntimeError("no route to the goal - a wall was probably misread")
            d = self.maze.best_direction(dist, self.x, self.y, self.heading)
            self.move(d)
            self.x += DX[d]
            self.y += DY[d]
            self.heading = d
            moves += 1
            if moves > max_moves:
                raise RuntimeError("gave up after %d moves" % max_moves)
            self.look()
        return moves

    def explore(self, goals, max_round_trips=4):
        # goal, back to start, repeat until the shortest route is proven (or out of trips)
        trips = 0
        while trips < max_round_trips:
            self.go_to(goals)
            self.go_to([self.start])
            trips += 1
            if self.maze.route_proven(self.start, goals):
                break
        return trips
