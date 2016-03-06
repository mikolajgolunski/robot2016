#!/bin/python3

import sys

import find_final_path


class Universe:
    """Provide ecosystem for the rest of the program.

    Fields:
    :robot: (Robot) robot existing in the Universe
    :board: (Board) board existing in the Universe
    :timeLimit: (float) time limit for the robot (default: None)
    """

    def __init__(self, robot, board):
        """Initialization method.

        :param robot: (Robot) class describing robot
        :param board: (Board) class describing board on which robot rides
        """
        self.robot = robot
        self.board = board
        self.timeLimit = None

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
                self.robot.noiseTurn = float(value)
            elif key == "distance_noise":
                self.robot.noiseMove = float(value)
            elif key == "forward_steering_drift":
                self.robot.noiseDrift = float(value)
            elif key == "speed":
                self.robot.vMove = float(value)
            elif key == "turning_speed":
                self.robot.vTurn = float(value)
            elif key == "M":
                self.board.dimensions["x"] = int(value)
            elif key == "N":
                self.board.dimensions["y"] = int(value)
            elif key == "execution_cpu_time_limit":
                self.timeLimit = float(value)
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


class Robot:
    """Describe robot.

    Fields:
    :position: () current position of the robot
    :vMove: (float) moving speed (default: 0.0)
    :vTurn: (float) turning speed (default: 0.0)
    :tickMove: (float) how far does it move in one tick (default: 0.01)
    :tickTurn: (float) how far does it turn in one tick (default: 0.002)
    :noiseMove: (float) sigma of the Gaussian noise applied to the distance during moving (default: 0.0)
    :noiseTurn: (float) sigma of the Gaussian noise applied to the angle during turning (default: 0.0)
    :noiseDrift: (float) angle added to the position during moving (default: 0.0)
    :color: (tuple) 3 tuple of ints storing color in RGB format (default: (None, None, None))
    :time: (float) time of the simulation
    """

    def __init__(self, position):  # TODO: Write position type
        """Initialization method.

        :param position: () -- current position of the robot
        """
        self.position = position
        self.vMove = 0.2
        self.vTurn = 1.0
        self.tickMove = 0.01
        self.tickTurn = 0.002
        self.noiseTurn = 0.0
        self.noiseMove = 0.0
        self.noiseDrift = 0.0
        self.color = (None, None, None)
        self.time = 0.0

    def read(self):
        """Read commands given by the simulator."""
        while True:
            key = sys.stdin.readline().strip()
            if key == "act":
                break
            elif key == "color":
                value = sys.stdin.readline().strip()
                self.color = tuple(map(int, value.split()))
            elif key == "time":
                value = sys.stdin.readline().strip()
                self.time = float(value)
            else:
                raise NotImplementedError("Unknown key")

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

if __name__ == "__main__":
    robot = Robot({"x": 2.5, "y": 2.5, "phi": 0.0})
    board = Board({"x": 9, "y": 9})
    u = Universe(robot, board)

    # Initial read from the simulator # TODO: Test reading of the simulator commands
    u.init_read()

    path = find_final_path.correct_final_path()
    commands = u.robot.command_generator(path)

    while True:
        # Read from the simulator performed at every step
        try:
            u.robot.read()
        except NotImplementedError:
            break

        try:
            sys.stdout.write(next(commands))
            sys.stdout.flush()
        except StopIteration:
            break

        # moving in straight line turning at walls
        # if 160 < robot.color[0] < 180:
        #     sys.stdout.write("TURN 1500\n")
        #     sys.stdout.flush()
        # else:
        #     sys.stdout.write("MOVE 1\n")
        #     sys.stdout.flush()
    sys.stdout.write("FINISH")
    sys.stdout.flush()
