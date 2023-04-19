#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from movimiento_puzzlebot.msg import set_point
from std_msgs.msg import String

vel = ""
ang = ""
a=0
k=0
r=1
z=0

an = rospy.get_param("an") 
di = rospy.get_param("di")

global i,j
i=0
j=0

def callback(msg):
    global vel,z,a
    vel=msg.data
    z=1
    a=a+1

def callback2(msg):
    global ang,k
    ang=msg.data
    k=1
def wr_callback(msg):
    global wr
    wr = msg.data

def wl_callback(msg):
    global wl
    wl = msg.data

def stop(self):
    print("Stopping")
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    w_pub.publish(msg)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")  
    wr = 0.0
    wl = 0.0
    rate = rospy.Rate(10)
    delim = ','
    sub = rospy.Subscriber('/help_msg', String, callback)
    sub2 = rospy.Subscriber('/help_msg2', String, callback2)

    

    #Variable initialisations
    distance = 0.0
    angle = 0.0
    current_time = rospy.get_time()
    last_time = rospy.get_time()
        
    # Create message for publishing
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    

    while not rospy.is_shutdown():

	if z==1 and k==1:
	    if r==1:	
 	        velocidad=vel.split(delim)
	        angulo=ang.split(delim)
	        vel=len(velocidad) 
	        ang=len(angulo)
	        r=0
	        #print(ang)
	    # Compute time since last loop
	    rospy.Subscriber('/wr',Float32,wr_callback)
            rospy.Subscriber('/wl',Float32,wl_callback)
	    w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time
	    distance += 0.05 * (wr + wl) * 0.5 * dt
            angle += 0.05 * (wr - wl) / 0.18 * dt
	    wr = 0
            wl = 0
	    # state 0 = moving along a straight
            msg.linear.x = float(velocidad[i])
            if i%2==0:
	    	msg.angular.z = 0.0
	    else: 
	        msg.angular.z = an

	    print(distance)
	    print(angle)
            #print(angulo[i])
	    print(i)
	    #print(j)
	    if float(angulo[i])==float(velocidad[i]):
	        i=i+1
	        j=j+1
	    
	    if distance > di:
                    #Reset distance
                    distance = 0.0
                    angle = 0.0
                    # If we have not finished the square
                    # Stop and signal
                    msg.linear.x = 0
                    msg.angular.z = 0
                    print("Motion Completed")
                    #rospy.signal_shutdown("Square Completed")
		    i=i+1
		    j=j+1
  	    if angle > float(angulo[i]) and j==1:
		    #Reset distance
                    distance = 0.0
		    angle = 0.0
                    # If we have not finished the square
                    # Stop and signal
                    msg.linear.x = 0
                    msg.angular.z = 0
                    print("Motion Completed")
                    #rospy.signal_shutdown("Square Completed")
		    i=i+1
		    j=j-1
            	

            w_pub.publish(msg)
            rate.sleep()           
