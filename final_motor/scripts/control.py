#!/usr/bin/env python
import rospy
import numpy as np
from final_motor.msg import set_point
from std_msgs.msg import Float32

#Variables
hz = 30
counter = 0
signal = 0.0

entry = 0.0
time = 0.0
motor_out = 0.0

error = 0
prev_e = 0
sum_e = 0
diff_e = 0

#Functions
def callback(msg):
    global entry, time
    entry = msg.input
    time = msg.time

def output(msg):
    global motor_out
    motor_out = msg.data

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("control")
    rate = rospy.Rate(hz)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    input_pub = rospy.Publisher("/motor_input", Float32, queue_size=1)
    rospy.Subscriber("/set_point", set_point, callback)
    rospy.Subscriber("/motor_output", Float32, output)

    print("The Control is Running")

    #Run the node
    while not rospy.is_shutdown():
	kp = rospy.get_param("/control_kp")
      	ki = rospy.get_param("/control_ki")
	kd = 0.0
	dt = rospy.get_param("/control_dt", 1.0/1000.0)
        #Write your code here
	if (counter <= hz*time):
	    error = entry - motor_out
	    sum_e += error * dt
	    diff_e = (error - prev_e) / dt
	    signal = kp * error + ki * sum_e + kd * diff_e
	    prev_e = error
	    counter += 1
	else:
	    signal = 0.0
	    entry = 0.0
	    counter = 0.0
	    prev_e = 0
	    sum_e = 0

        input_pub.publish(signal)
	rospy.loginfo("Set Point Velocity: %f rad/s", entry)
        rospy.loginfo("Motor input (mapped): %f", signal)

        rate.sleep()
