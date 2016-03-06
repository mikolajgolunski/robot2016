# import MGTiming
from math import pi, sin, cos, floor, ceil

# ----CELE----
# kazdy cel sklada sie z prostokatow
# kazdy prostokat wyznaczonym jest przez dwa zakresy: dla x i dla y
# cele z jednej grupy znajduja sie w jednej liscie
# wszystkie cele znajduja sie w jednej liscie grup celow

objective_r1 = (  # caly cel
    (  # pierwszy prostokat
        (4.1, 4.9),  # zakres x
        (2.1, 2.9)  # zakres y
    ),  # ewentualny drugi prostokat
)
objective_r2 = (
    ((4.1, 4.9), (4.1, 4.9)),
)

objective_g1 = (
    ((2.1, 2.9), (5.1, 5.9)),
)
objective_g2 = (
    ((6.1, 6.9), (6.1, 6.9)),
)

objective_b1 = (
    ((6.1, 6.9), (3.1, 3.9)),
)

objectives_r = (objective_r1, objective_r2)  # grupa celow czerwonych
objectives_g = (objective_g1, objective_g2)
objectives_b = (objective_b1,)

objectives = (objectives_r, objectives_g, objectives_b)

# ----PRZESZKODY----
# kazda przeszkoda sklada sie z listy prostokatow
# kazdy prostokat opisany jest dwoma zakresami: dla x i dla y

side_left = (  # cala przeszkoda
    (  # pierwszy prostokat
        (0.0, 1.1),  # zakres x
        (0.0, 9.0)  # zakres y
    ),  # ewentualny drugi prostokat, itd
)

side_right = (
    ((7.9, 9.0), (0.0, 9.0)),
)

side_up = (
    ((0.0, 9.0), (0.0, 1.1)),
)

side_down = (
    ((0.0, 9.0), (7.9, 9.0)),
)

obstacles = (side_left, side_right, side_up, side_down)

# ----OBSLUGA BLEDOW----


class CrashError(Exception):
    pass


class FoundIt(Exception):
    pass


class HaveIt(Exception):
    pass

# ----RESZTA----

position = {"x": 2.5, "y": 2.5, "phi": 0.0}
v_turn = 1.0
v_move = 0.2
tick_move = 0.01
tick_turn = 0.002
ticks_angle = 1 / tick_turn
ticks_distance = 1 / tick_move

noise_drift = 0.00008


class BranchNode:
    def __init__(self, node_position):
        self.position = node_position.copy()
        self.moves = []
        self.cost = 0.0
        self.objective_group_index = -1
        self.positions = []


class Branch:
    def __init__(self, position, cost, moves):
        self.moves = moves.copy()
        self.movement = {"move": 0.0, "turn": 0.0, "beep": False}
        self.position = position.copy()
        self.cost = cost
        self.objective_group_index = 0
        self.reverse = False
        self.positions = []


class Path:
    def __init__(self):
        self.cost = None
        self.moves = []
        self.limits = None
        self.positions = []

    def __str__(self):
        return str(self.cost) + ", [" + "; ".join(["{turn: " + str(move["turn"]) + ", move: " + str(move["move"]) +
                                                   "}" for move in self.moves]) + "]"


def simulate_route(position_in, moves_in):
    position_out = position_in.copy()
    position_out["phi"] += moves_in["turn"] * tick_turn
    if position_out["phi"] > 2 * pi:
        position_out["phi"] -= 2 * pi
    elif position_out["phi"] < 0:
        position_out["phi"] += 2 * pi
    if moves_in["move"] >= 0:
        for i in range(moves_in["move"]):
            position_out["x"] += tick_move * cos(position_out["phi"])
            position_out["y"] += tick_move * sin(-position_out["phi"])
            position_out["phi"] -= noise_drift
    else:
        for i in range(-moves_in["move"]):
            position_out["x"] -= tick_move * cos(position_out["phi"])
            position_out["y"] -= tick_move * sin(-position_out["phi"])
            position_out["phi"] += noise_drift
    return position_out


def find_path(path_in, movement_in, best_path, node_in):
    best_path.limits = movement_in
    if path_in is None:
        div_2pi = floor(2 * pi / movement_in["angle"])
        angles_list = [movement_in["angle"] * div for div in range(div_2pi)]
        distance_offset = 0.0
    else:
        path_moves = path_in.moves[node_in.objective_group_index + 1]
        div_nr = ceil(path_in.limits["angle"] * 2 / movement_in["angle"])
        angle_offset = path_moves["turn"] - (movement_in["angle"] * div_nr / 2)
        angles_list = [angle_offset + (movement_in["angle"] * div) for div in range(div_nr)]
        if path_moves["move"] >= 0:
            distance_offset = path_moves["move"] - path_in.limits["distance"]
        else:
            distance_offset = path_moves["move"] + path_in.limits["distance"]

    move_distance = movement_in["distance"]
    branches = []
    for angle in angles_list:
        if angle > pi:
            angle += -2 * pi
        elif angle < -pi:
            angle += 2 * pi
        distance_offset_ticks = int(distance_offset * ticks_distance)
        angle_ticks = int(angle * ticks_angle)
        branch = Branch(node_in.position, node_in.cost, node_in.moves)
        branch.objective_group_index = node_in.objective_group_index + 1
        branch.cost += abs(angle_ticks) / v_turn
        branch.cost += abs(distance_offset_ticks) / v_move
        branch.position = simulate_route(branch.position, {"move": distance_offset_ticks, "turn": angle_ticks})
        branch.movement["turn"] = angle
        branch.movement["move"] = distance_offset
        branches.append(branch)
    for angle in angles_list:
        if angle > pi:
            angle += -2 * pi
        elif angle < -pi:
            angle += 2 * pi
        distance_offset_ticks = int(distance_offset * ticks_distance)
        angle_ticks = int(angle * ticks_angle)
        branch = Branch(node_in.position, node_in.cost, node_in.moves)
        branch.reverse = True
        branch.objective_group_index = node_in.objective_group_index + 1
        branch.cost += abs(angle_ticks) / v_turn
        branch.cost += abs(distance_offset_ticks) / v_move
        branch.position = simulate_route(branch.position, {"move": distance_offset_ticks, "turn": angle_ticks})
        branch.movement["turn"] = angle
        branch.movement["move"] = distance_offset
        branches.append(branch)
    while True:
        interesting_branches = []
        for branch in branches:
            branch.cost += move_distance * ticks_distance / v_move
            if best_path.cost is not None and branch.cost > best_path.cost:
                continue

            if branch.reverse:
                simulated_moves = {"move": -int(move_distance * ticks_distance), "turn": 0}
                branch.movement["move"] -= move_distance
            else:
                simulated_moves = {"move": int(move_distance * ticks_distance), "turn": 0}
                branch.movement["move"] += move_distance
            branch.position = simulate_route(branch.position, simulated_moves)

            try:
                for obstacle in obstacles:
                    for rectangle in obstacle:
                        if rectangle[0][0] < branch.position["x"] < rectangle[0][1] \
                                and rectangle[1][0] < branch.position["y"] < rectangle[1][1]:
                            raise CrashError
            except CrashError:
                continue

            try:
                for objective in objectives[branch.objective_group_index]:
                    for rectangle in objective:
                        if rectangle[0][0] < branch.position["x"] < rectangle[0][1] \
                                and rectangle[1][0] < branch.position["y"] < rectangle[1][1]:
                            branch.movement["beep"] = True
                            branch.moves.append(branch.movement)
                            branch.positions.append(branch.position)
                            if branch.objective_group_index < len(objectives) - 1:
                                node = BranchNode(branch.position)
                                node.moves = branch.moves.copy()
                                node.positions = branch.positions.copy()
                                node.cost = branch.cost
                                node.objective_group_index = branch.objective_group_index
                                best_path = find_path(path_in, movement_in, best_path, node)
                                raise HaveIt
                            else:
                                raise FoundIt
            except FoundIt:
                if best_path.cost is None or branch.cost < best_path.cost:
                    best_path.cost = branch.cost
                    best_path.moves = branch.moves
                    best_path.positions = branch.positions
                continue
            except HaveIt:
                continue

            interesting_branches.append(branch)
        if len(interesting_branches) == 0:
            break
        branches = interesting_branches
    return best_path


def path_to_ticks(path, limits):
    path_out = []
    for move in path.moves:
        path_out.append({"move": ceil(move["move"] / limits["distance"]),
                         "turn": ceil(move["turn"] / limits["angle"]),
                         "beep": move["beep"]})
    return path_out


def create_final_path():
    movement = {"angle": 2*pi/32, "distance": 0.2}
    best_path = find_path(path_in=None, movement_in=movement, best_path=Path(), node_in=BranchNode(position))
    # print(best_path)

    movement = {"angle": 2*pi/360, "distance": 0.05}
    best_path = find_path(path_in=best_path, movement_in=movement, best_path=Path(), node_in=BranchNode(position))
    # print(best_path)

    movement = {"angle": tick_turn, "distance": tick_move}
    best_path = find_path(path_in=best_path, movement_in=movement, best_path=Path(), node_in=BranchNode(position))
    # print(best_path)

    best_path_ticks = path_to_ticks(best_path, movement)

    return best_path_ticks

# final = create_final_path()
