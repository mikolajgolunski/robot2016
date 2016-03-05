import fileinput


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

    def read(self, command):
        """Read commands given by the simulator.

        :param command: (string) command to act accordingly
        """
        for line in fileinput.input():
            line = line.strip()
            if command == "color":
                self.color = map(int, line.split())
            elif command == "time":
                self.time = float(line)
            else:
                raise RuntimeError("Unknown runtime command.")
            if command is None:
                yield line
            else:
                yield None

    def init_read(self, command):
        """Read initial commands given by the simulator.

        :param command: (string) command to act accordingly
        """
        for line in fileinput.input():
            line = line.strip()
            if command == "x":
                self.position["x"] = float(line)
            elif command == "y":
                self.position["y"] = float(line)
            elif command == "angle":
                self.position["phi"] = float(line)
            elif command == "steering_noise":
                self.noiseTurn = float(line)
            elif command == "distance_noise":
                self.noiseMove = float(line)
            elif command == "forward_steering_drift":
                self.noiseDrift = float(line)
            elif command == "speed":
                self.vMove = float(line)
            elif command == "turning_speed":
                self.vTurn = float(line)
            elif command == "M":
                u.board.dimensions["x"] = int(line)
            elif command == "N":
                u.board.dimensions["y"] = int(line)
            elif command == "execution_cpu_time_limit":
                u.timeLimit = float(line)
            else:
                raise RuntimeError("Unknown initialization command.")
            if command is None:
                yield line
            else:
                yield None

if __name__ == "__main__":
    robot = Robot({"x": 2.5, "y": 2.5, "phi": 0.0})
    board = Board({"x": 9, "y": 9})
    u = Universe(robot, board)

    # Initial read from the simulator
    command = None  # TODO: Test reading of the simulator commands
    while True:
        try:
            command = next(u.robot.init_read(command))
        except StopIteration:
            break

    # Read from the simulator performed at every step
    command = None
    while True:
        try:
            command = next(u.robot.read(command))
        except StopIteration:
            raise RuntimeError("Unexpected end of runtime read.")
        if command == "act":
            break
