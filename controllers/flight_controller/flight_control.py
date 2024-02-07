"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
#La ziz
class Motor():
    def __init__(self,robot:Robot):
        self.lwheel:Motor = robot.getDevice('left wheel motor')
        self.rwheel:Motor = robot.getDevice('right wheel motor')
        
    def backward(self):
        self.lwheel.setPosition(float('inf'))
        self.rwheel.setPosition(float('inf'))
        self.lwheel.setVelocity(-6.0)
        self.rwheel.setVelocity(-6.0)

    def turn_left(self):
        self.lwheel.setPosition(float('inf'))
        self.rwheel.setPosition(float('inf'))
        self.lwheel.setVelocity(-6.0)
        self.rwheel.setVelocity(0)

    def forward(self):
        self.lwheel.setPosition(float('inf'))
        self.rwheel.setPosition(float('inf'))
        self.lwheel.setVelocity(6.0)
        self.rwheel.setVelocity(6.0)
        
class monRobot(Robot):
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
        

    def run(self):

        if self.ps0.getValue() <= 100 or self.ps7.getValue() <= 100:
            self.motors.forward()
        if self.ps0.getValue() >= 100 or self.ps7.getValue() >= 100 or self.ps1.getValue() >= 100 or self.ps6.getValue() >= 100:
            self.motors.backward()
            self.motors.turn_left()
            self.motors.backward()
            self.motors.turn_left()

        

# create the Robot instance.
robot = monRobot()

# get the time step of the current world.


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(robot.timestep) != -1:

    robot.run()
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
