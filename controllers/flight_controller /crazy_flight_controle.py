#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from math import cos, sin

import sys
sys.path.append('../../../controllers')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    ## Initialize motors
    m1_motor = robot.getDevice("m1_motor");
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor");
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor");
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor");
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    ## Initialize Sensors
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    ## Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    ## Initialize variables

    past_x_global, past_y_global = (None,None)
    past_time = robot.getTime()

    # Crazyflie velocity PID controller
    PID_CF = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    # Velocity PID control (converted from Crazyflie c code)
    # Original ones
    gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
             "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}
    # Mine
    # gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
    #          "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 8, "ki_z": 5, "kd_z": 5}


    height_desired = FLYING_ATTITUDE

    print("\n");

    print("====== Controls =======\n\n");

    print(" The Crazyflie can be controlled from your keyboard!\n");
    print(" All controllable movement is in body coordinates\n");
    print("- Use the up, back, right and left button to move in the horizontal plane\n");
    print("- Use Q and E to rotate around yaw ");
    print("- Use W and S to go up and down\n ")
    
    it_idx = 0
    
    # Main loop:
    while robot.step(timestep) != -1:

        dt = robot.getTime() - past_time
        actual_state = {}

        ## Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        y_global = gps.getValues()[1]

        if it_idx == 0:
            # Initialization
            v_x_global = (x_global - x_global)/dt
            v_y_global = (y_global - y_global)/dt
            it_idx += 1
        
        else:
            v_x_global = (x_global - past_x_global)/dt
            v_y_global = (y_global - past_y_global)/dt
        
        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        ## Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        key = keyboard.getKey()
        while key>0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired =  + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1

            key = keyboard.getKey()


        height_desired += height_diff_desired * dt

        ## Example how to get sensor data
        ## range_front_value = range_front.getValue();
        ## cameraData = camera.getImage()


        ## PID velocity controller with fixed height
        motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y, gains)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global