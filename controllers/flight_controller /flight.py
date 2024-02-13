"""my_controller controller."""
#add comment 
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import numpy as np
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

    tubes = [[[-52, 73, 4] [-56, 80, 4]], #Violet
            [[-60, 72, 7] [-60, 67, 7]],  #Cyan
            [[-72, 60, 10] [-66, 60, 10]], # wrong
            [[-74, 70, 2] [-74, 82, 2]]]  #Jaune
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