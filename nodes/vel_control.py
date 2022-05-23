#!/usr/bin/env python3

from asyncio import queues
from cProfile import run
import rospy
import odrive
import time
import numpy as np
import matplotlib.pyplot as plt
from odrive_controller.msg import MotorVel
from pathlib import Path

class VelocityControl:
    def __init__(self):
        home_dir = str(Path.home())
        self.save_dir = home_dir + "/odrive_ws/src/odrive_controller/data/"
        print(self.save_dir)

        self.odrv0 = odrive.find_any()  # this will not work when using multiple odrives

        # Check calibration
        assert self.odrv0.axis1.encoder.config.pre_calibrated, "Need to calibrate axis1 encoder"
        assert self.odrv0.axis1.motor.config.pre_calibrated, "Need to calibrate axis1 motor"
        assert self.odrv0.axis0.encoder.config.pre_calibrated, "Need to calibrate axis0 encoder"
        assert self.odrv0.axis0.motor.config.pre_calibrated, "Need to calibrate axis0 motor"

        # Axis State Closed Loop Control
        self.odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

        self.odrv0.axis0.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL

        self.vel_pub_odrv0_ = rospy.Publisher("odrv0/current_vel", MotorVel, queue_size=1) # initializing publisher
        self.curr_vel_ = MotorVel() # initialize self.curr_vel_ to be the a MotorVel ROS msg
        self.curr_vel_.axis0_vel = self.odrv0.axis0.encoder.vel_estimate
        self.curr_vel_.axis1_vel = self.odrv0.axis1.encoder.vel_estimate
        print("Estimated values:\n", self.curr_vel_)

        # Set startup velocities to 2 rpm
        self.vel =  60.0
        self.odrv0.axis0.controller.input_vel = self.vel 
        self.odrv0.axis1.controller.input_vel = self.vel

        # time.sleep(5)

        print(self.vel)

        self.vel_sub_odrv0_ = rospy.Subscriber("odrv0/cmd_vel", MotorVel, self.set_vel_cb) # initializing sub

        # Set up lists of estimated values
        self.axis0_vel_est = []
        self.axis1_vel_est = []
        self.axis0_vel_req = []
        self.axis1_vel_req = []
    
    def set_vel_cb(self, motor_vel):
        self.odrv0.axis0.controller.input_vel = motor_vel.axis0_vel
        self.odrv0.axis1.controller.input_vel = motor_vel.axis1_vel
    
    def pub_curr_vel(self):
        # Get estimate of the velocity of each motors
        self.curr_vel_.axis0_vel = self.odrv0.axis0.encoder.vel_estimate
        self.curr_vel_.axis1_vel = self.odrv0.axis1.encoder.vel_estimate

        # if self.curr_vel_.axis0_vel != 0 and self.curr_vel_.axis1_vel != 0 :
        self.axis0_vel_est.append(self.curr_vel_.axis0_vel)
        self.axis1_vel_est.append(self.curr_vel_.axis1_vel)
        self.axis0_vel_req.append(self.vel)
        self.axis1_vel_req.append(self.vel)
        # double the velocity every 500 data points collected
        if len(self.axis0_vel_est)%201 == 0 and self.vel > 0:
            self.vel -= 4
            self.odrv0.axis0.controller.input_vel = self.vel 
            self.odrv0.axis1.controller.input_vel = self.vel
        elif self.vel < 0:
            rospy.signal_shutdown("end")

        print("\nAxis0 List: ", *self.axis0_vel_est)
        print("\nAxis1 List: ", *self.axis1_vel_est)

        print("\n\nEstimated values:\n", self.curr_vel_)
        self.vel_pub_odrv0_.publish(self.curr_vel_)

    def on_shutdown(self):
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
        
        # Set requested state to idle
        self.odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
        self.odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_IDLE

        est = np.array(self.axis0_vel_est)
        est2 = np.array(self.axis1_vel_est)
        req = np.array(self.axis0_vel_req)
        req2 = np.array(self.axis1_vel_req)

        velVals0 = np.vstack((est,req))
        velVals1 = np.vstack((est2,req2))
        
        np.save(self.save_dir + "VelValsAxis0", velVals0)
        np.save(self.save_dir + "VelValsAxis1", velVals1)

        plt.figure(1)
        plt.scatter(range(velVals0.shape[1]), velVals0[0, :])
        plt.scatter(range(velVals0.shape[1]), velVals0[1, :])
        plt.savefig("vel_data_axis0.png")

        plt.figure(2)
        plt.scatter(range(velVals1.shape[1]), velVals1[0, :])
        plt.scatter(range(velVals1.shape[1]), velVals1[1, :])
        plt.savefig("vel_data_axis1.png")

        plt.show()

        self.odrv0.axis0.controller.config.vel_integrator_gain
        

#connection to odrive
def run_odrive():
    rospy.init_node("odrive_vel_controller")
    rate = rospy.Rate(60)
    vel_ctrl = VelocityControl()
    rospy.on_shutdown(vel_ctrl.on_shutdown)
    while not rospy.is_shutdown():
        vel_ctrl.pub_curr_vel()
        rate.sleep()

if __name__ == '__main__':
    try:
        run_odrive()
    except rospy.ROSInterruptException:
        pass



# vel_error = vel_cmd - vel_feedback,
# current_integral += vel_error * vel_integrator_gain,
# current_cmd = vel_error * vel_gain + current_integral + current_feedforward
