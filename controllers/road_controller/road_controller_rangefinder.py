"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, RangeFinder

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
        

    
    def turn_left(self,n):
        self.front_lwheel.setPosition(float('inf'))
        self.front_rwheel.setPosition(float('inf'))
        self.back_lwheel.setPosition(float('inf'))
        self.back_rwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(n)
        self.front_rwheel.setVelocity(n*2)
        self.back_lwheel.setVelocity(n)
        self.back_rwheel.setVelocity(n*2)
        

    def turn_right(self,n):
        self.front_lwheel.setPosition(float('inf'))
        self.front_rwheel.setPosition(float('inf'))
        self.back_lwheel.setPosition(float('inf'))
        self.back_rwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(n*2)
        self.front_rwheel.setVelocity(n)
        self.back_lwheel.setVelocity(n*2)
        self.back_rwheel.setVelocity(n)


    def straight(self,n):
        self.front_lwheel.setPosition(float('inf'))
        self.front_rwheel.setPosition(float('inf'))
        self.back_lwheel.setPosition(float('inf'))
        self.back_rwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(n)
        self.front_rwheel.setVelocity(n)
        self.back_lwheel.setVelocity(n)
        self.back_rwheel.setVelocity(n)

    def backward(self,n):
        self.front_lwheel.setPosition(float('inf'))
        self.front_rwheel.setPosition(float('inf'))
        self.back_lwheel.setPosition(float('inf'))
        self.back_rwheel.setPosition(float('inf'))
        self.front_lwheel.setVelocity(-n)
        self.front_rwheel.setVelocity(-n)
        self.back_lwheel.setVelocity(-n)
        self.back_rwheel.setVelocity(-n)

    


class monRobot(Robot):
    def __init__(self):
        super().__init__()

        self.motors = Motors(self)
        self.Cam = AllCam(self)
        self.timestep = int(self.getBasicTimeStep())
        


    def run(self):

        self.motors.straight(5)

        # Sa valeur max est 8m
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
                DepthSommeOne = DepthSommeOne + DepthImage[j]

            for j in range(i-2*int(640/3),i-int(640/3)):
                DepthSommeTwo = DepthSommeTwo + DepthImage[j]

            for j in range(i-int(640/3),i):
                DepthSommeTree = DepthSommeTree + DepthImage[j]
            

        if DepthSommeOne >9000000 : 
            DepthSommeOne = 0

        if DepthSommeTwo >9000000 : 
            DepthSommeTwo = 0

        if DepthSommeTree >9000000 : 
            DepthSommeTree = 0

        DepthArraySomme = [int(DepthSommeOne),int(DepthSommeTwo),int(DepthSommeTree)]
        #DepthMax = max(DepthArraySomme)
        DepthMax = [i for i, j in enumerate(DepthArraySomme) if j == max(DepthArraySomme)]
        #print(type(DepthMax))

        match DepthMax[0]:
            case 0:
                self.motors.turn_left(5)
                print("bite")
            case 1:
                self.motors.straight(5)
            case 2:
                self.motors.turn_right(5)
   

        #DepthPixels = self.Cam.depth_cam.rangeImageGetDepth(DepthImage,DepthWidth,DepthWidth/2,DeptHeight/2)
        
        #print(DepthPixels)

        #print(self.depth_cam.getValue())






# create the Robot instance.
robot = monRobot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
robot.Cam.depth_cam.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    robot.run() #on avance, et on esquive les murs
    print("OHOHOH")
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
