"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, RangeFinder

class Motors():
    def __init__(self,robot:Robot):
        self.front_rwheel:Motor = robot.getDevice('front_right_wheel_link')
        self.front_lwheel:Motor = robot.getDevice('front_left_wheel_link')
        self.back_lwheel:Motor = robot.getDevice('back_left_wheel_link')
        self.back_lwheel:Motor = robot.getDevice('back_right_wheel_link')
        

    
    def turn_left(self):
        self.front_lwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(6)
        self.front_rwheel.setPosition(float('inf'))
        self.front_rwheel.setVelocity(2)
        

    def turn_right(self):
        self.front_lwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(2)
        self.front_rwheel.setPosition(float('inf'))
        self.front_rwheel.setVelocity(6)

    def straight(self):
        self.front_lwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(6)
        self.front_rwheel.setPosition(float('inf'))
        self.front_rwheel.setVelocity(6)

    def backward(self):
        self.front_lwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(-6)
        self.front_rwheel.setPosition(float('inf'))
        self.front_rwheel.setVelocity(-6)

class monRobot(Robot):
    def __init__(self):
        super().__init__()

        self.motors = Motors(self)
        self.timestep = int(self.getBasicTimeStep())
        self.ps0:RangeFinder = self.getDevice('depth_camera')
        self.ps0.enable(self.timestep)
        self.ps7.enable(self.timestep)

    flag_wall = 0

    def run(self):

        

        print(self.ps0.getValue())

        if self.ps0.getValue() >= 80 and self.ps7.getValue() >= 80 or flag_wall == 1:
            
            flag_wall = 1
            print(flag_wall)
            self.motors.backward()
            self.motors.turn_right()
            flag_wall = 0
            
        elif flag_wall == 0: 
            self.motors.straight()




# create the Robot instance.
robot = monRobot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    robot.run() #on avance, et on esquive les murs
    print("AHAHAHAH")
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
