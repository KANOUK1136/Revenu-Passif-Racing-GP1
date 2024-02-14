"""
my_controller controller.
"""
import time
import math
from controller import Robot, Motor, Camera, Lidar, Accelerometer, Gyro, Compass, DistanceSensor

TIME_STEP = 32
MAX_VELOCITY = 26
base_speed = 18

turn_duration = 1
left_turn_speed = -15
right_turn_speed = 15


# Set empirical coefficients for collision avoidance
coefficients = [[17.0, -9.0], [-17.0, 9.0]]

# Initialize the robot
robot = Robot()

# Get motor devices
front_left_motor = robot.getDevice("fl_wheel_joint")
front_right_motor = robot.getDevice("fr_wheel_joint")
rear_left_motor = robot.getDevice("rl_wheel_joint")
rear_right_motor = robot.getDevice("rr_wheel_joint")

# Set target position to infinity (speed control)
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))

# Get position sensors and enable them
front_left_position_sensor = robot.getDevice("front left wheel motor sensor")
front_right_position_sensor = robot.getDevice("front right wheel motor sensor")
rear_left_position_sensor = robot.getDevice("rear left wheel motor sensor")
rear_right_position_sensor = robot.getDevice("rear right wheel motor sensor")
front_left_position_sensor.enable(TIME_STEP)
front_right_position_sensor.enable(TIME_STEP)
rear_left_position_sensor.enable(TIME_STEP)
rear_right_position_sensor.enable(TIME_STEP)

# Get IMU devices and enable them
accelerometer = robot.getDevice("imu accelerometer")
gyro = robot.getDevice("imu gyro")
compass = robot.getDevice("imu compass")
accelerometer.enable(TIME_STEP)
gyro.enable(TIME_STEP)
compass.enable(TIME_STEP)

# Get distance sensors and enable them
distance_sensors = [robot.getDevice("rl_range"), robot.getDevice("fl_range"),
                    robot.getDevice("fr_range"), robot.getDevice("rr_range")]
for sensor in distance_sensors:
    sensor.enable(TIME_STEP)

front_left_motor.setVelocity(left_turn_speed)
rear_left_motor.setVelocity(left_turn_speed)
front_right_motor.setVelocity(right_turn_speed)
rear_right_motor.setVelocity(right_turn_speed)

for _ in range(turn_duration):
    robot.step(TIME_STEP)

front_left_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

# Main loop
while robot.step(TIME_STEP) != -1:

    # Get distance sensor values
    distance_sensors_value = [sensor.getValue() for sensor in distance_sensors]
    
    # Compute motors speed
    avoidance_speed = [0.0, 0.0]
    motor_speed = [0.0, 0.0]  # Define motor_speed as a list
    for i in range(2):
        avoidance_speed[i] = 0.0
        for j in range(1, 3):
            avoidance_speed[i] += (2.0 - distance_sensors_value[j]) * (2.0 - distance_sensors_value[j]) * coefficients[i][j - 1]
        motor_speed[i] = base_speed + avoidance_speed[i]
        motor_speed[i] = min(motor_speed[i], MAX_VELOCITY)
        motor_speed[i] = max(motor_speed[i], -MAX_VELOCITY)

    # Set speed values
    print(motor_speed[0])
    print(motor_speed[1])
    
    front_left_motor.setVelocity(motor_speed[0])
    front_right_motor.setVelocity(motor_speed[1])
    rear_left_motor.setVelocity(motor_speed[0])
    rear_right_motor.setVelocity(motor_speed[1])
