"""my_controller controller."""
#add comment 
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, DistanceSensor, Motor


class Motor ():
    
    def __init__(self,robot:Robot):
        self.lwheel:Motor = robot.getDevice('left wheel motor')
        self.rwheel:Motor = robot.getDevice('right wheel motor')

class monDrone(Robot):

    def __init__(self):
        super().__init__()
        self.motors = Motor(self)

        self.timestep = int(self.getBasicTimeStep())

        self.ps0:DistanceSensor  = self.getDevice('ps0')
        self.ps1:DistanceSensor  = self.getDevice('ps1')
        self.ps6:DistanceSensor  = self.getDevice('ps6')
        self.ps7:DistanceSensor  = self.getDevice('ps7')

        self.ps0.enable(self.timestep)
        self.ps7.enable(self.timestep)
        self.ps1.enable(self.timestep)
        self.ps6.enable(self.timestep)

# create the Robot instance.
robot = monDrone()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.


def in_out_tube() : 

    tubes = [[[-49.7024,71.6138,3.95],[-57.2266,79.9015,3.95]],
            [[-60.6527,74.8525,6.91],[-60.6527,66.2525,6.91]],
            [[-62.4127,59.6225,9.82],[-73.0927,59.6925,9.91]],
            [[-74.0827,68.5625,1.88],[-74.0827,82.1125,1.88]]]
    for i in range(0,4):
        print(tubes[i])

    direction = []
    tubes_order = [1,4,3,2]
    for i in tubes_order :
        direction.append(tubes[i-1])
    print()
    for i in range(0,4):
        print(direction[i])
in_out_tube()