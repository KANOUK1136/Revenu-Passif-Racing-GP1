"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import robot, motor

class Motors():

    def __init__(self, robot):
        # Assigner une variable à un objet réel pour le contrôler 
        self.flwheel:motor = robot.getDevice('front left wheel')
        self.frwheel:motor = robot.getDevice('front right wheel')
        self.blwheel:motor = robot.getDevice('rear left wheel')
        self.brwheel:motor = robot.getDevice('rear right wheel')

    def forward(self):
        self.flwheel.wb_set_motor_position(float('inf'))
        self.flwheel.wb_set_motor_velocity(100)
        self.frwheel.wb_set_motor_position(float('inf'))
        self.frwheel.wb_set_motor_velocity(100)
        self.blwheel.wb_set_motor_position(float('inf'))
        self.blwheel.wb_set_motor_velocity(100)
        self.brwheel.wb_set_motor_position(float('inf'))
        self.brwheel.wb_set_motor_velocity(100)


class monRobot(robot):

    def __init__(self):
        super().__init__()
        self.motors = Motors(self)
        self.timestep = int(self.getBasicTimeStep())
       

    def run(self):
        self.motors.forward()
            

# create the Robot instance.
rosbot_1 = monRobot()

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while rosbot_1.step(rosbot_1.timestep) != -1:

    rosbot_1.run() # On avance et on esquive les murs 

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
