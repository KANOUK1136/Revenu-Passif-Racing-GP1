"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, RangeFinder, Camera, GPS

import time


    
def calc_coeff(val):

    if val > 120000:
        120000
    
    coef = round(val * 10 / 12000, 2)

    if coef >60:
        coef = 60
    
    if coef <10:
        coef = 10
    return coef

class AllCam():
    def __init__(self,robot:Robot):
        self.depth_cam:RangeFinder = robot.getDevice('depth_camera')

    def getMaxRange(self):
        self.timestep = int(robot.getBasicTimeStep())
        self.depth_cam.enable(self.timestep)
        max = self.depth_cam.getMaxRange()
        return max
    
class Motors():
    def __init__(self,robot:Robot):
        self.front_rwheel:Motor = robot.getDevice('front_right_wheel_joint')
        self.front_lwheel:Motor = robot.getDevice('front_left_wheel_joint')

        self.back_lwheel:Motor = robot.getDevice('back_left_wheel_joint')
        self.back_rwheel:Motor = robot.getDevice('back_right_wheel_joint')

        self.front_lwheel.setPosition(float('inf'))
        self.front_rwheel.setPosition(float('inf'))
        self.back_lwheel.setPosition(float('inf'))
        self.back_rwheel.setPosition(float('inf'))
    
    def turn_left(self,vit):

        self.front_lwheel.setVelocity(vit/5)
        self.back_lwheel.setVelocity(vit/5)
        self.front_rwheel.setVelocity(vit)
        self.back_rwheel.setVelocity(vit)

    def turn_right(self,vit):
  
        self.front_lwheel.setVelocity(vit)
        self.back_lwheel.setVelocity(vit)
        self.front_rwheel.setVelocity(vit/5)
        self.back_rwheel.setVelocity(vit/5)

    def forward(self,vit):

        self.front_lwheel.setVelocity(vit)
        self.front_rwheel.setVelocity(vit)
        self.back_lwheel.setVelocity(vit)
        self.back_rwheel.setVelocity(vit)

    def backward(self,vit):

        self.front_lwheel.setVelocity(-vit)
        self.front_rwheel.setVelocity(-vit)
        self.back_lwheel.setVelocity(-vit)
        self.back_rwheel.setVelocity(-vit)

class monRobot(Robot):

    def __init__(self):
        super().__init__()

        self.motors = Motors(self)
        self.timestep = int(self.getBasicTimeStep())
        self.ps0:RangeFinder = self.getDevice('depth_camera')
        self.Cam = AllCam(self)
        self.cam:Camera = self.getDevice('rgb_camera')
        self.gps:GPS = self.getDevice("gps")
        
        self.x = self.gps.getValues()[0]
        self.y = self.gps.getValues()[1]

        self.runtime = time.time()

    def run(self):

        self.x = self.gps.getValues()[0]
        self.y = self.gps.getValues()[1]

        print(self.x)
        print(self.y)

        if self.x < -10 and self.x > -13 and self.y > 13.5 and self.y < 15.8:
            self.motors.front_lwheel.setVelocity(30)
            self.motors.back_lwheel.setVelocity(30)

            self.motors.front_rwheel.setVelocity(19)
            self.motors.back_rwheel.setVelocity(19)
            print("je suis a la fin")
        
        elif  self.y > 15.8 and self.x < -13:
            self.motors.forward(80)
        else :
            Max = self.Cam.getMaxRange()

            # DepthWidth = 640
            DepthWidth = self.Cam.depth_cam.getWidth()

            # DeptHeight = 480
            DeptHeight = self.Cam.depth_cam.getHeight()

            # L'image getRangeImageArray est une liste de taille 480
            # L'image getRangeImage est une liste de taille 307200
            DepthImage = self.Cam.depth_cam.getRangeImage(data_type='list')
            
            # On crop la moitié supérieur de l'image
            DepthImage = DepthImage[int(len(DepthImage)/2) :]

            DepthSommeOne = 0
            DepthSommeTwo= 0
            DepthSommeTree = 0

            for i in range(640,len(DepthImage),640) :
                for j in range(i-640,i-2*int(640/3)):
                    if DepthImage[j] >9000000 : 
                        DepthImage[j] = 0

                    DepthSommeOne = DepthSommeOne + DepthImage[j]

                for j in range(i-2*int(640/3),i-int(640/3)):
                    if DepthImage[j] >9000000 : 
                        DepthImage[j] = 0

                    DepthSommeTwo = DepthSommeTwo + DepthImage[j]

                for j in range(i-int(640/3),i):
                    if DepthImage[j] >9000000 : 
                        DepthImage[j] = 0
                    DepthSommeTree = DepthSommeTree + DepthImage[j]

            if DepthSommeTwo >9000000 : 
                DepthSommeTwo = 0

            if DepthSommeTree >9000000 : 
                DepthSommeTree = 0

            DepthArraySomme = [int(DepthSommeOne),int(DepthSommeTwo),int(DepthSommeTree)]
            DepthMax = max(DepthArraySomme)
            print("Val max", DepthMax)
            coef = calc_coeff(DepthMax)
            
            DepthMax = [i for i, j in enumerate(DepthArraySomme) if j == max(DepthArraySomme)]
            print("Coef",coef)

            match DepthMax[0]:
                case 0:
                    self.motors.turn_left(coef)

                case 1:
                    self.motors.forward(coef)

                case 2:
                    self.motors.turn_right(coef)

        



# create the Robot instance.
robot = monRobot()

# get the time step of the current world.
robot.cam.enable(robot.timestep)
robot.gps.enable(robot.timestep)
robot.cam.recognitionEnable(robot.timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller

while robot.step(robot.timestep) != -1:

    robot.run() 
    
    #on avance, et on esquive les murs
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.