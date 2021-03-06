#!/usr/bin/env python3

# ---IMPORTS---
import copy
import sys
from math import pi, sin, cos, sqrt, atan2
# import MGTiming


# ---CUSTOM ERRORS---
class CrashError(Exception):
    pass


class FoundIt(Exception):
    pass


class HaveIt(Exception):
    pass


# ---HELPER FUNCTIONS---
def my_round(x, base=1.0):
    return base * round(x / base)


def my_copy(obj, use_deepcopy=False):
    dignore = {str: None, int: None, type(None): None}

    t = type(obj)

    if t in (list, tuple):
        if t == tuple:
            # Convert to a list if a tuple to
            # allow assigning to when copying
            is_tuple = True
            obj = list(obj)
        else:
            # Otherwise just do a quick slice copy
            obj = obj[:]
            is_tuple = False

        # Copy each item recursively
        for x in range(len(obj)):
            if type(obj[x]) in dignore:
                continue
            obj[x] = my_copy(obj[x], use_deepcopy)

        if is_tuple:
            # Convert back into a tuple again
            obj = tuple(obj)

    elif t == dict:
        # Use the fast shallow dict copy() method and copy any
        # values which aren't immutable (like lists, dicts etc)
        obj = obj.copy()
        for k in obj:
            if type(obj[k]) in dignore:
                continue
            obj[k] = my_copy(obj[k], use_deepcopy)

    elif t in dignore:
        # Numeric or string/unicode?
        # It's immutable, so ignore it!
        pass

    elif use_deepcopy:
        obj = copy.deepcopy(obj)
    return obj


# ---HELPER CLASSES---
class BranchNode:
    def __init__(self, node_position):
        self.position = node_position.copy()
        self.moves = []
        self.cost = 0.0
        self.objective_group_index = -1
        self.positions = []


class Branch:
    def __init__(self, position, cost, moves):
        self.moves = my_copy(moves)
        self.movement = {"move": 0.0, "turn": 0.0, "beep": False}
        self.position = position.copy()
        self.cost = cost
        self.objective_group_index = 0
        self.reverse = False
        self.positions = []

    def copy(self):
        new = Branch(self.position, self.cost, self.moves)
        new.movement = self.movement.copy()
        new.objective_group_index = self.objective_group_index
        new.reverse = self.reverse
        new.positions = my_copy(self.positions)
        return new


class Path:
    def __init__(self):
        self.cost = None
        self.moves = []
        self.limits = None
        self.positions = []

    def __str__(self):
        return str(self.cost) + ", [" + "; ".join(["{turn: " + str(move["turn"]) + ", move: " + str(move["move"]) +
                                                   "}" for move in self.moves]) + "]"


# ---MAIN PART---
class Universe:
    """Provide ecosystem for the rest of the program.

    Fields:
    :robot: (Robot) robot existing in the Universe
    :board: (Board) board existing in the Universe
    :time_limit: (float) time limit for the robot (default: None)
    """

    def __init__(self, robot, board):
        """Initialization method.

        :param robot: (Robot) class describing robot
        :param board: (Board) class describing board on which robot rides
        """
        self.robot = robot
        self.board = board
        self.time_limit = None

    def init_read(self):
        """Read initial commands given by the simulator."""
        for i in range(11):
            line = sys.stdin.readline()
            key, value = line.strip().split(":")
            if key == "x":
                self.robot.position["x"] = float(value)
            elif key == "y":
                self.robot.position["y"] = float(value)
            elif key == "angle":
                self.robot.position["phi"] = float(value)
            elif key == "steering_noise":
                self.robot.noise_turn = float(value)
            elif key == "distance_noise":
                self.robot.noise_move = float(value)
            elif key == "forward_steering_drift":
                self.robot.noise_drift = float(value)
            elif key == "speed":
                self.robot.v_move = float(value)
            elif key == "turning_speed":
                self.robot.v_turn = float(value)
            elif key == "M":
                self.board.dimensions["x"] = int(value)
            elif key == "N":
                self.board.dimensions["y"] = int(value)
            elif key == "execution_cpu_time_limit":
                self.time_limit = float(value)
            else:
                raise NotImplementedError("Unknown initialization key.")


class Board:
    """Describe board.

    Fields:
    :dimensions: (dict) dimensions of the board:
        :"x": (int) x dimension
        :"y": (int) y dimension
    """

    def __init__(self, dimensions):
        """Initialization method.

        :param dimensions: (dict) dimensions of the board:
            :"x": (int) x dimension
            :"y": (int) y dimension
        """
        self.dimensions = dimensions
        self.obstacles = None
        self.objectives = [[], [], []]

    def initialize(self):
        # ----OBSTACLES----
        # each obstacle consists of rectangles
        # each rectangle consists of two ranges: x and y
        side_left = [  # whole obstacle
            [  # first rectangle
                [0.0, 1.1],  # x range
                [0.0, 9.0]  # y range
            ]  # possible second rectangle, and so on
        ]
        side_right = [
            [[7.9, 9.0], [0.0, 9.0]]
        ]
        side_up = [
            [[0.0, 9.0], [0.0, 1.1]]
        ]
        side_down = [
            [[0.0, 9.0], [7.9, 9.0]]
        ]
        self.obstacles = [side_left, side_right, side_up, side_down]

    def testing(self):
        # ---OBJECTIVES---
        easy = False
        if easy:
            obj_red = [[3, 2], [4, 2]]
            obj_green = [[5, 2]]
            obj_blue = [[5, 3]]
            self.objectives = [obj_red, obj_green]
        else:
            obj_red = [[3, 6], [6, 6]]
            obj_green = [[2, 2], [3, 4], [5, 4]]
            obj_blue = [[6, 2], [2, 6]]
            self.objectives = [obj_red, obj_green, obj_blue]


class Robot:
    """Describe robot.

    Fields:
    :position: () current position of the robot
    :v_move: (float) moving speed (default: 0.0)
    :v_turn: (float) turning speed (default: 0.0)
    :tick_move: (float) how far does it move in one tick (default: 0.01)
    :tick_turn: (float) how far does it turn in one tick (default: 0.002)
    :noise_move: (float) sigma of the Gaussian noise applied to the distance during moving (default: 0.0)
    :noise_turn: (float) sigma of the Gaussian noise applied to the angle during turning (default: 0.0)
    :noise_drift: (float) angle added to the position during moving (default: 0.0)
    :color: (tuple) 3 tuple of ints storing color in RGB format (default: (None, None, None))
    :time: (float) time of the simulation
    """

    def __init__(self, position):  # TODO: Write position type
        """Initialization method.

        :param position: () -- current position of the robot
        """
        self.position = position
        self.v_move = 0.2
        self.v_turn = 1.0
        self._tick_move = 0.01
        self._tick_turn = 0.002
        self.ticks_distance = 1 / self.tick_move
        self.ticks_angle = 1 / self.tick_turn
        self.noise_turn = 4e-6
        self.noise_move = 1e-5
        self.noise_drift = 2e-3
        self.color = (None, None, None)
        self.time = 0.0
        self.correction_table = []
        self.correction_table_reversed = {}

    @property
    def tick_move(self):
        return self._tick_move

    @tick_move.setter
    def tick_move(self, tick_move):
        self._tick_move = tick_move
        self.ticks_distance = 1 / self._tick_move

    @property
    def tick_turn(self):
        return self._tick_turn

    @tick_turn.setter
    def tick_turn(self, tick_turn):
        self._tick_turn = tick_turn
        self.ticks_angle = 1 / self._tick_turn

    def read(self, watching=True):
        """Read commands given by the simulator.

        :param watching: if True saves data to the robot, if False only reads them without saving
        """
        while True:
            key = sys.stdin.readline().strip()
            if key == "act":
                break
            elif watching:
                if key == "color":
                    value = sys.stdin.readline().strip()
                    self.color = tuple(map(int, value.split()))
                elif key == "time":
                    value = sys.stdin.readline().strip()
                    self.time = float(value)
                else:
                    raise NotImplementedError("Unknown key")

    @staticmethod
    def convert_objectives(board, offset):
        objectives_out = []
        for group in board.objectives:
            group_out = []
            for objective in group:
                objective_out = []
                for axis in objective:
                    objective_out.append([axis + offset, axis + 1 - offset])
                group_out.append(objective_out)
            objectives_out.append(group_out)
        board.objectives = objectives_out

    def create_correction_tables(self):
        iterator = 0
        temp_position = {"x": 0, "y": 0, "phi": 0}
        self.correction_table.append({"l": 0, "phi": 0})
        self.correction_table_reversed[round(0, 3)] = {"l": 0, "phi": 0}
        while iterator < 800:
            temp_position["x"] += self.tick_move * cos(temp_position["phi"])
            temp_position["y"] += self.tick_move * sin(-temp_position["phi"])
            temp_position["phi"] -= self.noise_drift
            l_new = sqrt(temp_position["x"]**2 + temp_position["y"]**2)
            phi_new = atan2(0, 1) - atan2(temp_position["y"] / l_new, temp_position["x"] / l_new)
            self.correction_table.append({"l": l_new, "phi": phi_new})
            self.correction_table_reversed[round(l_new, 3)] = {"l": iterator + 1, "phi": phi_new}
            iterator += 1

    def find_path(self, path_in, movement_in, best_path, node_in, board):
        best_path.limits = movement_in

        if path_in is None:
            ang = 0.0
            angles_list = [ang]
            while ang < pi / 2 - movement_in["angle"]:
                ang += my_round(movement_in["angle"], base=self.tick_turn)
                angles_list.append(ang)
            ang = 0.0
            while ang > -pi / 2 + movement_in["angle"]:
                ang -= my_round(movement_in["angle"], base=self.tick_turn)
                angles_list.append(ang)
            distance_offset = 0
        else:
            path_moves = path_in.moves[node_in.objective_group_index + 1]

            ang = path_moves["turn"]
            angles_list = [ang]
            while ang < path_moves["turn"] + path_in.limits["angle"] - movement_in["angle"]:
                ang += my_round(movement_in["angle"], base=self.tick_turn)
                angles_list.append(ang)
            ang = path_moves["turn"]
            while ang > path_moves["turn"] - path_in.limits["angle"] + movement_in["angle"]:
                ang -= my_round(movement_in["angle"], base=self.tick_turn)
                angles_list.append(ang)

            if path_moves["move"] >= 0:
                distance_offset = my_round(path_moves["move"] - path_in.limits["distance"], base=self.tick_move)
            else:
                distance_offset = my_round(path_moves["move"] + path_in.limits["distance"], base=self.tick_move)

        move_distance = my_round(movement_in["distance"], base=self.tick_move)

        branches = []
        for angle in angles_list:
            distance_offset_ticks = int(round(abs(distance_offset) * self.ticks_distance))

            offset_correction = self.correction_table[distance_offset_ticks]

            branch = Branch(node_in.position, node_in.cost, node_in.moves)
            branch.positions = copy.copy(node_in.positions)
            branch.objective_group_index = node_in.objective_group_index + 1

            if distance_offset < 0:
                branch.reverse = True

            if branch.objective_group_index > 1:
                branch.cost += abs(angle) / self.v_turn
            if branch.objective_group_index > 0:
                branch.cost += abs(distance_offset) / self.v_move

            if distance_offset >= 0:
                pm = +1
            else:
                pm = -1

            branch.position["phi"] += angle
            branch.position["x"] += pm * offset_correction["l"] * cos(branch.position["phi"] + offset_correction["phi"])
            branch.position["y"] += pm * offset_correction["l"] * sin(-(branch.position["phi"] + offset_correction["phi"]))
            branch.position["phi"] -= distance_offset_ticks * self.noise_drift

            branch.movement["turn"] = angle
            branch.movement["move"] = distance_offset
            branches.append(branch)

        if path_in is None:
            for angle in angles_list:
                distance_offset_ticks = int(round(abs(distance_offset) * self.ticks_distance))

                offset_correction = self.correction_table[distance_offset_ticks]

                branch = Branch(node_in.position, node_in.cost, node_in.moves)
                branch.positions = copy.copy(node_in.positions)
                branch.objective_group_index = node_in.objective_group_index + 1
                branch.reverse = True

                if branch.objective_group_index > 1:
                    branch.cost += abs(angle) / self.v_turn
                if branch.objective_group_index > 0:
                    branch.cost += abs(distance_offset) / self.v_move

                if distance_offset >= 0:
                    pm = +1
                else:
                    pm = -1

                branch.position["phi"] += angle
                branch.position["x"] += pm * offset_correction["l"] * cos(branch.position["phi"] + offset_correction["phi"])
                branch.position["y"] += pm * offset_correction["l"] * sin(
                    -(branch.position["phi"] + offset_correction["phi"]))
                branch.position["phi"] -= distance_offset_ticks * self.noise_drift

                branch.movement["turn"] = angle
                branch.movement["move"] = distance_offset
                branches.append(branch)

        # for branch in branches[:]:
        #     new_branch = branch.copy()
        #     new_branch.reverse = True
        #     branches.append(new_branch)
        # reverse_branches = my_copy(branches, True)
        # for branch in reverse_branches:
        #     branch.reverse = False
        # branches.extend(reverse_branches)

        while True:
            interesting_branches = []
            for branch in branches:
                tick_distance = int(round(move_distance * self.ticks_distance))
                if branch.objective_group_index > 0:
                    branch.cost += abs(move_distance) / self.v_move
                if best_path.cost is not None and branch.cost > best_path.cost:
                    continue

                correction = self.correction_table[tick_distance]
                if branch.reverse:
                    pm = -1
                else:
                    pm = +1
                branch.movement["move"] += pm * move_distance
                branch.position["x"] += pm * correction["l"] * cos(branch.position["phi"] + correction["phi"])
                branch.position["y"] += pm * correction["l"] * sin(-(branch.position["phi"] + correction["phi"]))
                branch.position["phi"] -= tick_distance * self.noise_drift

                try:
                    for obstacle in board.obstacles:
                        for rectangle in obstacle:
                            if rectangle[0][0] < branch.position["x"] < rectangle[0][1] \
                                    and rectangle[1][0] < branch.position["y"] < rectangle[1][1]:
                                raise CrashError
                except CrashError:
                    continue

                try:
                    if path_in is not None and path_moves["move"] >= 0 and branch.movement["move"] > \
                            my_round(path_moves["move"] + path_in.limits["distance"], base=self.tick_move):
                        continue
                    elif path_in is not None and path_moves["move"] < 0 and branch.movement["move"] < \
                            my_round(path_moves["move"] - path_in.limits["distance"], base=self.tick_move):
                        continue
                    for objective in board.objectives[branch.objective_group_index]:
                        if objective[0][0] < branch.position["x"] < objective[0][1] \
                                and objective[1][0] < branch.position["y"] < objective[1][1]:
                            branch.movement["beep"] = True
                            branch.moves.append(branch.movement)
                            branch.positions.append(branch.position)
                            if branch.objective_group_index < len(board.objectives) - 1:
                                node = BranchNode(copy.copy(branch.position))
                                node.moves = my_copy(branch.moves)
                                node.positions = copy.copy(branch.positions)
                                node.cost = branch.cost
                                node.objective_group_index = branch.objective_group_index
                                best_path = self.find_path(path_in, movement_in, best_path, node, board)

                                branch.moves_temp = branch.moves[:-1]
                                branch.moves = branch.moves_temp[:]
                                branch.positions_temp = branch.positions[:-1]
                                branch.positions = branch.positions_temp[:]

                                # raise HaveIt
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

    @staticmethod
    def path_to_ticks(path, limits):
        path_out = []
        for move_nr, move in enumerate(path.moves):
            path_out.append({"move": int(round(move["move"] / limits["distance"])),
                             "turn": int(round(move["turn"] / limits["angle"])),
                             "position": path.positions[move_nr],
                             "beep": move["beep"]})
        return path_out

    def create_final_path(self, board):
        movement = {"angle": 2*pi/75, "distance": 0.1}  # rough check
        best_path = self.find_path(
            path_in=None,
            movement_in=movement,
            best_path=Path(),
            node_in=BranchNode(self.position),
            board=board)
        # print(best_path)

        movement = {"angle": 2*pi/360, "distance": 0.02}  # refine path
        best_path = self.find_path(
            path_in=best_path,
            movement_in=movement,
            best_path=Path(),
            node_in=BranchNode(self.position),
            board=board)
        # print(best_path)

        movement = {"angle": self.tick_turn, "distance": self.tick_move}  # completely refine path
        best_path = self.find_path(
            path_in=best_path,
            movement_in=movement,
            best_path=Path(),
            node_in=BranchNode(self.position),
            board=board)
        # print(best_path)

        # convert path in distance and angle units to ticks
        best_path_ticks = self.path_to_ticks(best_path, {"angle": self.tick_turn, "distance": self.tick_move})
        # for p in best_path_ticks:
        #     print(p)
        #     pass
        return best_path_ticks

    def correct_final_path(self, board):
        path = self.create_final_path(board)
        # if len(path) > 1:
        #     position = path[0]["position"].copy()
        #     position["phi"] += path[1]["turn"] * self.tick_turn
        #     temp_position = position.copy()
        #     counter_ticks = 0
        #     if path[1]["move"] >= 0:
        #         pm = +1
        #     else:
        #         pm = -1
        #     while True:
        #         position["x"] += pm * self.tick_move * cos(position["phi"])
        #         position["y"] += pm * self.tick_move * sin(-position["phi"])
        #         position["phi"] -= pm * self.noise_drift
        #         try:
        #             for objective in board.objectives[0]:
        #                 if objective[0][0] < position["x"] < objective[0][1] \
        #                         and objective[1][0] < position["y"] < objective[1][1]:
        #                     counter_ticks += pm * 1
        #                     temp_position = position.copy()
        #                     raise FoundIt
        #             break
        #         except FoundIt:
        #             pass
        moves = {"move": 0, "turn": path[1]["turn"], "position": path[1]["position"], "beep": True}
        moves["position"]["phi"] += path[1]["turn"] * self.tick_turn
        path[0]["beep"] = False
        path[1]["turn"] = 0
        path.insert(1, moves)
        # print("---CORRECTION---")
        # for p in path:
        #     print(p)
        #     pass
        return path

    @staticmethod
    def command_generator(path):
        """Generate commands from path.

        :param path:
        :return:
        """
        for move in path:
            commands = ["TURN " + str(move["turn"]) + "\n", "MOVE " + str(move["move"]) + "\n"]
            if move["beep"]:
                commands.append("BEEP\n")
            for command in commands:
                yield command

    # makes a movement
    # x - shift in ticks
    def move_forward(self, l, start_angle, ticks):
        # znajdz dist i angle w tabelce Mikiego - nieaktualne, do funkcji podajemy juz przeliczona wartosc
        dist = ticks

        if dist >= 0:
            angle2 = dist * (self.noise_drift/self.tick_turn) - start_angle
        else:
            angle2 = -dist * (self.noise_drift/self.tick_turn) - start_angle
            # print("angle2:" + str(angle2) + "dist:" + str(dist) + "start_angle:" + str(start_angle))

        angle2 = int(angle2)

        # poczekaj na act
        self.read(watching=False)
        # obrot o angle
        sys.stdout.write("TURN " + str(start_angle) + "\n")
        sys.stdout.flush()
        # poczekaj na act
        self.read(watching=False)
        # idz dist i obroc sie nazot
        sys.stdout.write("MOVE " + str(dist) + "\n")
        sys.stdout.flush()
        # poczekaj na act
        self.read(watching=False)
        # idz dist i obroc sie nazot
        sys.stdout.write("TURN " + str(angle2) + "\n")
        sys.stdout.flush()

        # refresh robot position
        self.position["x"] += l * cos(self.position["phi"])
        self.position["y"] += l * sin(-self.position["phi"])

    # makes a rotation movement
    # phi - angle in ticks
    def move_rotate(self, phi):
        # waiting for "act"
        self.read(watching=False)
        # rotate
        sys.stdout.write("TURN " + str(phi) + "\n")
        sys.stdout.flush()

        # refreash robot position
        self.position["phi"] += phi * self.tick_turn

    # test the actual color under sensor
    # x,y - actual robot position (int, as table indexes)
    def test_field_color(self, x, y, universe):

        # read the color value
        self.read(watching=True)

        # test the current color
        r = self.color[0]
        g = self.color[1]
        b = self.color[2]

        # print("coooooooooooolor:" + str(r) + "-" + str(g) + "-" + str(b))

        sys.stdout.write("TURN 0 \n")
        sys.stdout.flush()
        # send any command

        # if white
        if g > 210 and r > 200 and b > 200:
            return "1"

        # if red
        elif r > 210 and g < 50 and b < 50:
            # write to table
            universe.board.objectives[0].append([x, y])
            # print("reeeeeeed:" + str(r) + "-" + str(g) + "-" + str(b))
            # print_board(universe.board.objectives)
            return "r"
        # if green
        elif g > 110 and r < 50 and b < 100:
            # write to table
            universe.board.objectives[1].append([x, y])
            # print("greeeeeen" + str(r) + "-" + str(g) + "-" + str(b))
            # print_board(universe.board.objectives)
            return "g"
        # if blue
        elif b > 100 and r < 80 and g < 100:
            # write to table
            universe.board.objectives[2].append([x, y])
            # print("blueeeee" + str(r) + "-" + str(g) + "-" + str(b))
            # print_board(universe.board.objectives)
            return "b"
        # if wall
        elif r > 100:
            return "w"
        # if black
        else:
            return "0"

    def scan_fields(self, x1, y1, x2, y2, scan_angle, universe):
        self.move_rotate(int(scan_angle/2))
        self.test_field_color(x1, y1, universe)
        self.move_rotate(-scan_angle)
        self.test_field_color(x2, y2, universe)
        self.move_rotate(int(scan_angle/2))


# ---PROPER EXECUTION PART---
if __name__ == "__main__":
    def initialize_world():
        robot = Robot({"x": 2.5, "y": 2.5, "phi": 0.0})
        board = Board({"x": 9, "y": 9})
        universe = Universe(robot, board)

        if not local_testing:
            universe.init_read()  # Initial read from the simulator
        universe.board.initialize()  # Board initialization
        universe.robot.create_correction_tables()

        if testing:
            universe.board.testing()  # Actions on board for testing purposes
        return universe

    def final_path_commands(universe):
        universe.robot.convert_objectives(universe.board, offset=0.1)

        # path = universe.robot.create_final_path(universe.board)
        path = universe.robot.correct_final_path(universe.board)
        commands = universe.robot.command_generator(path)
        return commands

    def robot_write(universe, commands, watching=True):
        while True:
            # Read from the simulator performed at every step
            universe.robot.read(watching)

            try:
                # Write next command
                sys.stdout.write(next(commands))
                sys.stdout.flush()
            except StopIteration:
                break

    def format_row(row):
        return '[' + ','.join(str(x) for x in row) + ']'

    def format_colour_table(board):
        return ''.join(format_row(row) for row in board)

    def print_board(board):
        b = "TABLICA:["
        c = "]_[".join(format_colour_table(row) for row in board)
        b += c + "]KONIEC \n"
        print(b)

    def find_colours(universe):
        # scanning angle (ticks)
        scan_angle = 1280

        # initialize tick values for required movements
        mov = {}
        mov[100] = {"l": 100, "phi": 0, "ticks": 666}
        mov[50] = {"l": 50,  "phi": 0, "ticks": 666}

        # saerch for tick values for required movements
        diff100 = 99999
        diff50 = 99999

        for (i, item) in enumerate(universe.robot.correction_table):
            if i < 150:
                # for 50
                d = abs(0.5 - item["l"])
                if d < diff50:
                    diff50 = d
                    mov[50] = {"l": item["l"],  "phi": int(-item["phi"] / universe.robot.tick_turn), "ticks": i}
                # for 100
                d = abs(1.0 - item["l"])
                if d < diff100:
                    diff100 = d
                    mov[100] = {"l": item["l"],  "phi": int(-item["phi"] / universe.robot.tick_turn), "ticks": i}

        # mov[50]["phi"] = mov[100]["phi"]

            # t +=  str(i)+":" + str(item["l"])+":"+str(item["phi"])+"|"

        # t = "100=l:"+ str( mov[100]["l"]) + ",phi:" + str(mov[100]["phi"]) + ",ticks:" + str(mov[100]["ticks"]) + "|"
        # t += "50=l:"+ str(mov[50]["l"]) + ",phi:" + str(mov[50]["phi"]) + ",ticks:" + str(mov[50]["ticks"]) + "|"

        # print(t)

        # ##################################################################
        # KLAUD's CRAZY MOVEMENTS START
        # universe.robot.move_rotate(-1570)
        universe.robot.move_forward(-mov[50]["l"], mov[50]["phi"], -mov[50]["ticks"])
        # universe.robot.move_forward(mov[50]["l"], mov[50]["phi"], mov[50]["ticks"])
        # universe.robot.move_rotate(1570)

        # universe.robot.move_forward(-mov[100]["l"], -mov[100]["phi"], -mov[100]["ticks"])
        # universe.robot.move_forward(mov[50]["l"], mov[50]["phi"], mov[50]["ticks"])
        # universe.robot.move_forward(-mov[50]["l"], -mov[50]["phi"], -mov[50]["ticks"])
        universe.robot.test_field_color(2, 2, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.test_field_color(3, 2, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.test_field_color(4, 2, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.test_field_color(5, 2, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.test_field_color(6, 2, universe)
        universe.robot.move_rotate(-785)
        universe.robot.move_forward(mov[50]["l"], mov[50]["phi"], mov[50]["ticks"])
        universe.robot.scan_fields(6, 3, 5, 3, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.move_rotate(int(scan_angle/2))
        universe.robot.test_field_color(6, 4, universe)
        universe.robot.move_rotate(-scan_angle)
        universe.robot.test_field_color(5, 4, universe)
        universe.robot.move_rotate(int(-(785-scan_angle/2)))
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(4, 4, 4, 3, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(3, 4, 3, 3, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        # print(str(universe.robot.position["x"]) + str(universe.robot.position["y"]))
        universe.robot.move_rotate(int(-scan_angle/2))
        universe.robot.test_field_color(2, 3, universe)
        universe.robot.move_rotate(scan_angle)
        universe.robot.test_field_color(2, 4, universe)
        universe.robot.move_rotate(int(785-scan_angle/2))
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(3, 5, 2, 5, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.move_rotate(int(-scan_angle/2))
        universe.robot.test_field_color(2, 6, universe)
        universe.robot.move_rotate(scan_angle)
        universe.robot.test_field_color(3, 6, universe)
        universe.robot.move_rotate(int(785-scan_angle/2))
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(4, 5, 4, 6, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(5, 5, 5, 6, scan_angle, universe)
        universe.robot.move_forward(mov[100]["l"], mov[100]["phi"], mov[100]["ticks"])
        universe.robot.scan_fields(6, 5, 6, 6, scan_angle, universe)
        # KLAUD's CRAZY MOVEMENTS END
        # ##################################################################


    local_testing = False
    testing = False

    universe = initialize_world()

    # universe.robot.move_rotate(-500)
    # universe.robot.move_forward(190)
    # print("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT:" + universe.robot.test_field_color(3,6, universe) + "||\n")
    # universe.robot.test_field_color(1,1, universe)
    # print_board(universe.board.objectives)

    find_colours(universe)
    # sys.stdout.write("BEEP\n")
    # sys.stdout.flush()
    # print(str(universe.robot.position["x"]) + "," + str(universe.robot.position["y"]))
    # print_board(universe.board.objectives)

    # pos = "position:x" + str(universe.robot.position["x"]) + ",y" + str(universe.robot.position["y"] )+ ",phi" + str(universe.robot.position["phi"]) + "|"
    # print(pos)

    # universe.robot.position["x"] = 2.5
    # universe.robot.position["y"] = 2.5

    # sys.stdout.write("BEEP\n")
    # sys.stdout.flush()

    # import cProfile
    # cProfile.run('final_path_commands(universe)', sort=1)

    commands = final_path_commands(universe)  # finding final path
    if not local_testing:
        robot_write(universe, commands, watching=False)

    # moving in straight line turning at walls # TODO: move to separate place
    # if 160 < robot.color[0] < 180:
    #     sys.stdout.write("TURN 1500\n")
    #     sys.stdout.flush()
    # else:
    #     sys.stdout.write("MOVE 1\n")
    #     sys.stdout.flush()
    sys.stdout.write("FINISH")
    sys.stdout.flush()
