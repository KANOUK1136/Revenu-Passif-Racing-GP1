"""my_controller_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Motor,DistanceSensor
from time import sleep
class Motors():
    def __init__(self,robot:Robot):
        self.lwheel:Motor = robot.getDevice('left wheel motor')
        self.rwheel:Motor = robot.getDevice('right wheel motor')
        
    def turn_left(self):
       
        self.lwheel.setPosition(float('inf')) 
        self.rwheel.setVelocity(6.28) 
        self.rwheel.setPosition(float('inf')) 
        self.rwheel.setVelocity(-6.28)
        
    def turn_right(self):
       
        self.lwheel.setPosition(-float('inf')) 
        self.rwheel.setVelocity(-6.28)  
        self.rwheel.setPosition(float('inf')) 
        self.rwheel.setVelocity(6.28)  
        

    def strait(self):  
        self.lwheel.setPosition(float('inf')) 
        self.rwheel.setVelocity(6.28) 
        self.rwheel.setPosition(float('inf')) 
        self.rwheel.setVelocity(6.28) 
         
    def backward(self):  
        self.lwheel.setPosition(100) 
        self.rwheel.setVelocity(-6.28) 
        self.rwheel.setPosition(100) 
        self.rwheel.setVelocity(-6.28) 
        
   
class monRobot(Robot):
    def __init__(self):
        super().__init__()

        self.motors =  Motors(self)

        self.timestep = int(self.getBasicTimeStep())
        self.ps0:DistanceSensor = self.getDevice('ps0')
        self.ps0.enable(self.timestep )
        self.ps7:DistanceSensor = self.getDevice('ps7')
        self.ps7.enable(self.timestep )
       
    def run(self):

        left = self.ps0.getValue()
        right = self.ps0.getValue()

        while left>80 or right >80 :
            print("a")
            self.motors.backward()
            left = self.ps0.getValue()
            right = self.ps0.getValue()  
        self.motors.strait()
        
# create the Robot instance.
robot = monRobot()

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
   
    robot.run()
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
   

# Enter here exit cleanup code.
