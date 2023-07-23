from motor_test import test_motor
import time
from pymavlink import mavutil


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)
        step += 0.2

def letter_A():
    # run_motors_timed(mav_connection, seconds = 2.5, motor_settings=[100, -100, -100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 10, motor_settings=[-100, -100, 100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 3.2, motor_settings=[100, -100, -100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 1, motor_settings=[0, 0, 0, 0, 0, 0])
    run_motors_timed(mav_connection, seconds = 9, motor_settings=[-100, -100, 100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 2, motor_settings=[50, 50, -50, -50, 0, 0])
    run_motors_timed(mav_connection, seconds = 0.3, motor_settings=[100, -100, -100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 4.5, motor_settings=[100, -100, 100, -100, 0, 0])
    run_motors_timed(mav_connection, seconds = 3.8, motor_settings=[-100, 100, -100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 1, motor_settings=[0, 0, 0, 0, 0, 0])
    run_motors_timed(mav_connection, seconds = 0.3, motor_settings=[-100, 100, 100, -100, 0, 0])
    run_motors_timed(mav_connection, seconds = 3, motor_settings=[-100, -100, 100, 100, 0, 0])

def letter_V():
    run_motors_timed(mav_connection, seconds = 10, motor_settings=[-100, -100, 100, 100, 0, 0])
    run_motors_timed(mav_connection, seconds = 2.9, motor_settings=[-100, 100, 100, -100, 0, 0])
    run_motors_timed(mav_connection, seconds = 0.4, motor_settings=[0, 0, 0, 0, 0, 0])
    run_motors_timed(mav_connection, seconds = 10, motor_settings=[-100, -100, 100, 100, 0, 0])

def letter_U():
    run_motors_timed(mav_connection, seconds=7, motor_settings=[-100,-100 ,100 ,100, 0, 0])
    run_motors_timed(mav_connection, seconds=7, motor_settings=[-100, -85, 100, 85, 0, 0])
    run_motors_timed(mav_connection, seconds=0.5, motor_settings=[-100, 100, 100, -100, 0, 0])
    run_motors_timed(mav_connection, seconds=8.4, motor_settings=[-100,-100 ,100 ,100, 0, 0])

def forward(seconds):
    run_motors_timed(mav_connection, seconds, motor_settings=[-100,-100 ,100 ,100, 0, 0])

def amongus():
    forward(6) #forward
    run_motors_timed(mav_connection, seconds=10, motor_settings=[-85,-100 ,85 ,100, 0, 0]) #right
    forward(6)
    run_motors_timed(mav_connection, seconds=2.5, motor_settings=[0, 100 , 0 ,-100, 0, 0]) #rotate left without moving
    run_motors_timed(mav_connection, seconds=25, motor_settings=[-98,-100 ,98 ,100, 0, 0])
    

    forward(20)
    run_motors_timed(mav_connection, seconds=9.5, motor_settings=[-85,-100 ,85 ,100, 0, 0]) #right leg
    forward(2.5)
    run_motors_timed(mav_connection, seconds=7, motor_settings=[-100,-75 ,100 ,75, 0, 0]) #left turn

    forward(2.5)
    run_motors_timed(mav_connection, seconds=9.5, motor_settings=[-85,-100 ,85 ,100, 0, 0])

    forward(12.5)
    run_motors_timed(mav_connection, seconds=2.2, motor_settings=[100, 0 , -100 ,0, 0, 0]) #rotate right without moving

    #finishes lens
    forward(2)
    run_motors_timed(mav_connection, seconds=10, motor_settings=[-100,-85 ,100 ,85, 0, 0])
    forward(5)


if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    

    letter_A()

    run_motors_timed(mav_connection, seconds=2, motor_settings=[0, 0, 0, 0, 0, 0]) # pause
    run_motors_timed(mav_connection, seconds=1.7, motor_settings=[-100, 100, 100, -100, 0, 0]) # turn
    run_motors_timed(mav_connection, seconds=4.5, motor_settings=[-100, -100, 50, 50, 0, 0]) # forward
    run_motors_timed(mav_connection, seconds=1.7, motor_settings=[-100, 100, 100, -100, 0, 0]) # turn
    run_motors_timed(mav_connection, seconds=9, motor_settings=[-100, -100, 50, 50, 0, 0]) # forward
    run_motors_timed(mav_connection, seconds=2, motor_settings=[0, 0, 0, 0, 0, 0]) # pause
    run_motors_timed(mav_connection, seconds=3.5, motor_settings=[-100, 100, 100, -100, 0, 0]) # turn
    letter_U()
    run_motors_timed(mav_connection, seconds=4, motor_settings=[100, -100, -100, 100, 0, 0]) # turn

    letter_V()


    #crab right rotate left
    run_motors_timed(mav_connection, seconds=23, motor_settings=[100, -100, 100, -100, 0, 0])
    run_motors_timed(mav_connection, seconds=2.5, motor_settings=[0, 100 , 0 ,-100, 0, 0])

    amongus()
    

    
    # stop
    run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 0, 0, 0, 0, 0])

    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)