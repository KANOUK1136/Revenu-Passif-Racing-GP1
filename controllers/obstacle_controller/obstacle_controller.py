"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera

class Motors():

    def __init__(self, robot:Robot):
        # Assigner une variable à un objet réel pour le contrôler 
        self.lwheel:Motor = robot.getDevice('left wheel motor')
        self.rwheel:Motor = robot.getDevice('right wheel motor')

    def forward(self):
        self.lwheel.setPosition(float('inf'))
        self.lwheel.setVelocity(6)
        self.rwheel.setPosition(float('inf'))
        self.rwheel.setVelocity(6)



class monRobot(Robot):

    def __init__(self):
        super().__init__()
        self.motors = Motors(self)
        
        # Assigner une variable aux capteurs de position pour les contrôler 
        self.ps0:DistanceSensor = self.getDevice('ps0')
        self.ps7:DistanceSensor = self.getDevice('ps7')
        self.camera:Camera = self.getDevice('camera')

        # Activer les capteurs
        self.timestep = int(self.getBasicTimeStep())
        self.ps0.enable(self.timestep)
        self.ps0.enable(self.timestep)


    def run(self):

        print("value ", self.camera.value)
        
        if (self.ps0.getValue() >= 80  or self.ps7.getValue() >= 80):
            if self.ps0.getValue() >= self.ps7.getValue() :
                self.motors.turn_left()
            else :
                self.motors.turn_right()
        else : 
            self.motors.forward()
            

# create the Robot instance.
robot = monRobot()


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(robot.timestep) != -1:

    robot.run() # On avance et on esquive les murs 


    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
