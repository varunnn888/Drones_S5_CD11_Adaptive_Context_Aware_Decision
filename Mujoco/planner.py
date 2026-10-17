# ============================================================
# ACMFDS AUTONOMOUS QUADROTOR
# MULTI-PATH ADAPTIVE PLANNER
# ============================================================

import heapq
import math


RES = 0.25
MINX, MINY = -2.0, -2.0
MAXX, MAXY = 14.5, 12.5
MARGIN = 0.55

OBSTACLES = [
    # x, y, half-width, half-depth, height (metres); spread across the expanded arena.
    (3.1, 2.8, 0.72, 0.62, 3.6),
    (5.2, 6.6, 0.78, 0.72, 4.0),
    (8.4, 3.6, 0.78, 0.64, 3.8),
    (9.3, 8.0, 0.68, 0.78, 3.3),
    (5.0, 9.0, 0.62, 0.56, 3.0),
    (7.1, 1.7, 0.60, 0.60, 3.0),
]


def obstacle_distance(x, y, obstacles=None):
    obstacles = OBSTACLES if obstacles is None else obstacles
    if not obstacles:
        return 999.0

    return min(
        math.hypot(
            max(abs(x - ox) - hx, 0.0),
            max(abs(y - oy) - hy, 0.0),
        )
        for ox, oy, hx, hy, _ in obstacles
    )


# Backward-compatible name.
def obstacle_clearance(x, y, obstacles=None):
    return obstacle_distance(x, y, obstacles)


def clearance_at(x, y, obstacles=None):
    return obstacle_distance(x, y, obstacles)


def blocked(x, y, obstacles=None):
    obstacles = OBSTACLES if obstacles is None else obstacles

    return any(
        abs(x - ox) <= hx + MARGIN
        and abs(y - oy) <= hy + MARGIN
        for ox, oy, hx, hy, _ in obstacles
    )


def idx(point):
    return (
        round((float(point[0]) - MINX) / RES),
        round((float(point[1]) - MINY) / RES),
    )


def world(grid_point):
    return (
        MINX + grid_point[0] * RES,
        MINY + grid_point[1] * RES,
    )


def segment_points(a, b, step=0.18):
    distance = math.dist(a, b)
    count = max(1, int(math.ceil(distance / step)))

    return [
        (
            a[0] + (b[0] - a[0]) * i / count,
            a[1] + (b[1] - a[1]) * i / count,
        )
        for i in range(count + 1)
    ]


def path_is_safe(path, obstacles=None):
    obstacles = OBSTACLES if obstacles is None else obstacles

    if len(path) < 1:
        return False

    for a, b in zip(path, path[1:]):
        for point in segment_points(a, b):
            if blocked(point[0], point[1], obstacles):
                return False

    return not blocked(path[-1][0], path[-1][1], obstacles)


def path_length(path):
    return sum(
        math.dist(a, b)
        for a, b in zip(path, path[1:])
    )


def path_min_clearance(path, obstacles=None):
    obstacles = OBSTACLES if obstacles is None else obstacles

    values = []
    for a, b in zip(path, path[1:]):
        for x, y in segment_points(a, b, step=0.25):
            values.append(obstacle_distance(x, y, obstacles))

    if not values and path:
        values.append(
            obstacle_distance(path[0][0], path[0][1], obstacles)
        )

    return min(values) if values else 0.0


def _nearest_valid(node, valid):
    if valid(node):
        return node

    for radius in range(1, 40):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                q = (node[0] + dx, node[1] + dy)
                if valid(q):
                    return q

    raise RuntimeError("No valid grid point found")


def plan_xy(
    start,
    goal,
    obstacles=None,
    clearance_weight=0.08,
    preference=0.0,
):
    """
    A* planner.

    `preference` creates a family of alternative routes:
      negative -> prefer the lower side of the map
      zero     -> balanced route
      positive -> prefer the upper side of the map

    The rest of the planner remains collision-aware.
    """
    obstacles = OBSTACLES if obstacles is None else obstacles

    s = idx(start)
    g = idx(goal)

    def valid(node):
        x, y = world(node)
        return (
            MINX <= x <= MAXX
            and MINY <= y <= MAXY
            and not blocked(x, y, obstacles)
        )

    s = _nearest_valid(s, valid)
    g = _nearest_valid(g, valid)

    # Reference line used to create alternative route "families".
    sx, sy = world(s)
    gx, gy = world(g)
    denom = max(abs(gx - sx), 1e-6)

    pq = [(0.0, s)]
    came_from = {}
    cost_so_far = {s: 0.0}

    directions = [
        (dx, dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if (dx, dy) != (0, 0)
    ]

    while pq:
        _, current = heapq.heappop(pq)

        if current == g:
            break

        for dx, dy in directions:
            neighbor = (
                current[0] + dx,
                current[1] + dy,
            )

            if not valid(neighbor):
                continue

            x, y = world(neighbor)
            cx, cy = world(current)
            # Prevent diagonal corner-cutting. Both adjacent orthogonal cells
            # must be free, and the actual swept segment must also be clear.
            # Without this edge check, a newly moved obstacle could be missed
            # even when both diagonal endpoints happen to be valid.
            if dx != 0 and dy != 0:
                if not valid((current[0] + dx, current[1])):
                    continue
                if not valid((current[0], current[1] + dy)):
                    continue
            if not path_is_safe([(cx, cy), (x, y)], obstacles):
                continue

            clearance = obstacle_distance(x, y, obstacles)

            # Base motion cost.
            move_cost = math.hypot(dx, dy)

            # Safer routes get lower cost.
            safety_cost = clearance_weight / max(clearance, 0.20)

            # Route-family preference.
            t = max(0.0, min(1.0, (x - sx) / denom))
            direct_y = sy + (gy - sy) * t

            # The sin term peaks near the middle and returns to zero
            # at start/end, so every candidate still starts/ends properly.
            preferred_y = direct_y + preference * math.sin(math.pi * t)

            family_cost = 0.055 * abs(y - preferred_y)

            new_cost = (
                cost_so_far[current]
                + move_cost
                + safety_cost
                + family_cost
            )

            if (
                neighbor not in cost_so_far
                or new_cost < cost_so_far[neighbor]
            ):
                cost_so_far[neighbor] = new_cost

                heuristic = math.hypot(
                    g[0] - neighbor[0],
                    g[1] - neighbor[1],
                )

                heapq.heappush(
                    pq,
                    (new_cost + heuristic, neighbor),
                )

                came_from[neighbor] = current

    if g != s and g not in came_from:
        raise RuntimeError("No collision-free path found")

    path = [g]
    while path[-1] != s:
        path.append(came_from[path[-1]])

    path.reverse()

    points = [world(n) for n in path]

    # Greedy line-of-sight simplification: only skip intermediate grid points
    # when the straight connector remains outside every obstacle's safety margin.
    # Simply dropping points by distance can cut across obstacle corners, which
    # becomes particularly visible when dynamic obstacles move between replans.
    simplified = [points[0]]
    current_index = 0
    while current_index < len(points) - 1:
        next_index = current_index + 1
        for candidate_index in range(len(points) - 1, current_index, -1):
            connector = [points[current_index], points[candidate_index]]
            if path_is_safe(connector, obstacles):
                next_index = candidate_index
                break
        simplified.append(points[next_index])
        current_index = next_index

    if simplified[-1] != points[-1]:
        simplified.append(points[-1])
    return simplified


def generate_candidate_paths(
    start,
    goal,
    obstacles=None,
):
    obstacles = OBSTACLES if obstacles is None else obstacles

    # These produce distinct route families.
    preferences = [
        ("LOWER", -3.0),
        ("LOWER-MID", -2.0),
        ("SLIGHT-LOW", -1.0),
        ("BALANCED", 0.0),
        ("SLIGHT-HIGH", 1.0),
        ("UPPER-MID", 2.0),
        ("UPPER", 3.0),
    ]

    candidates = []
    seen = set()

    for name, preference in preferences:
        # Use slightly different safety bias so route families respond
        # differently to risk without changing the candidate meaning.
        clearance_weight = 0.045 + 0.025 * abs(preference)

        try:
            path = plan_xy(
                start,
                goal,
                obstacles=obstacles,
                clearance_weight=clearance_weight,
                preference=preference,
            )
        except RuntimeError:
            continue

        if not path_is_safe(path, obstacles):
            continue

        signature = tuple(
            (round(float(x), 2), round(float(y), 2))
            for x, y in path
        )

        if signature in seen:
            continue

        seen.add(signature)

        candidates.append({
            "profile": name,
            "preference": preference,
            "path": path,
        })

    return candidates


def _turn_penalty(path):
    if len(path) < 3:
        return 0.0

    turns = 0.0

    for a, b, c in zip(path, path[1:], path[2:]):
        v1 = (
            b[0] - a[0],
            b[1] - a[1],
        )
        v2 = (
            c[0] - b[0],
            c[1] - b[1],
        )

        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)

        if n1 < 1e-9 or n2 < 1e-9:
            continue

        cosine = (
            v1[0] * v2[0] + v1[1] * v2[1]
        ) / (n1 * n2)

        cosine = max(-1.0, min(1.0, cosine))
        angle = math.acos(cosine)

        if angle > 0.45:
            turns += angle

    return min(1.0, turns / 8.0)


def evaluate_path(
    path,
    obstacles=None,
    weights=None,
    battery=100.0,
    speed=2.0,
):
    obstacles = OBSTACLES if obstacles is None else obstacles

    if weights is None:
        weights = {
            "safety": 0.20,
            "energy": 0.20,
            "time": 0.20,
            "tracking": 0.20,
            "mission": 0.20,
        }

    length = path_length(path)
    clearance = path_min_clearance(path, obstacles)

    safety = 1.0 - min(1.0, max(0.0, clearance / 2.5))
    energy = min(1.0, length / 16.0)

    estimated_time = length / max(speed, 0.8)
    time = min(1.0, estimated_time / 14.0)

    tracking = _turn_penalty(path)

    start = path[0]
    destination = path[-1]

    direct = max(
        math.dist(start, destination),
        1e-6,
    )

    mission = min(
        1.0,
        max(0.0, (length - direct) / direct),
    )

    # Low score = best route.
    score = (
        float(weights["safety"]) * safety
        + float(weights["energy"]) * energy
        + float(weights["time"]) * time
        + float(weights["tracking"]) * tracking
        + float(weights["mission"]) * mission
    )

    return {
        "score": float(score),
        "length": float(length),
        "clearance": float(clearance),
        "safety": float(safety),
        "energy": float(energy),
        "time": float(time),
        "tracking": float(tracking),
        "mission": float(mission),
        "waypoints": len(path),
    }


def rank_paths(
    start,
    goal,
    obstacles=None,
    weights=None,
    battery=100.0,
    speed=2.0,
):
    obstacles = OBSTACLES if obstacles is None else obstacles

    candidates = generate_candidate_paths(
        start,
        goal,
        obstacles,
    )

    ranked = []

    for path_index, candidate in enumerate(candidates, start=1):
        metrics = evaluate_path(
            candidate["path"],
            obstacles=obstacles,
            weights=weights,
            battery=battery,
            speed=speed,
        )

        ranked.append({
            "rank": 0,
            "path_id": path_index,
            "profile": candidate["profile"],
            "preference": candidate["preference"],
            "path": candidate["path"],
            **metrics,
        })

    ranked.sort(
        key=lambda item: (
            item["score"],
            -item["clearance"],
            item["length"],
        )
    )

    for rank, item in enumerate(ranked, start=1):
        item["rank"] = rank

    return ranked


def select_best_path(
    start,
    goal,
    obstacles=None,
    weights=None,
    battery=100.0,
    speed=2.0,
):
    ranking = rank_paths(
        start,
        goal,
        obstacles=obstacles,
        weights=weights,
        battery=battery,
        speed=speed,
    )

    if not ranking:
        raise RuntimeError("No candidate paths available")

    return ranking[0]["path"], ranking


def get_best_path(
    start,
    goal,
    obstacles=None,
    weights=None,
    battery=100.0,
    speed=2.0,
):
    path, _ = select_best_path(
        start,
        goal,
        obstacles=obstacles,
        weights=weights,
        battery=battery,
        speed=speed,
    )
    return path


def nearest_obstacle_distance(x, y, obstacles=None):
    return obstacle_distance(x, y, obstacles)


if __name__ == "__main__":
    start = (0.0, 0.0)
    goal = (8.0, 7.0)

    weights = {
        "safety": 0.30,
        "energy": 0.15,
        "time": 0.15,
        "tracking": 0.15,
        "mission": 0.25,
    }

    ranking = rank_paths(
        start,
        goal,
        obstacles=OBSTACLES,
        weights=weights,
        battery=100.0,
        speed=2.0,
    )

    print("=" * 72)
    print("ACMFDS MULTI-PATH PLANNER TEST")
    print("=" * 72)

    for item in ranking:
        print(
            f"Rank #{item['rank']} | "
            f"Path {item['path_id']} | "
            f"{item['profile']:<11} | "
            f"Score {item['score']:.3f} | "
            f"Length {item['length']:.2f} m | "
            f"Clearance {item['clearance']:.2f} m | "
            f"Waypoints {item['waypoints']}"
        )

    if ranking:
        print("\nSELECTED: Rank #1")
        print(ranking[0]["path"])
