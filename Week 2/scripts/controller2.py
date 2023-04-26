#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from movimiento_puzzlebot.msg import set_point
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D


a=0
k=0
r=1
z=0
h=0

q=0

n=0
o=0

l = 0.19
m = 0.05

ultima_medicion_a = 0.0
ultima_medicion_d = 0.0

error_acumulado_a = 0.0
error_acumulado_d = 0.0
first = True

pose = Pose2D()
pose.x = 0.0
pose.y = 0.0
pose.theta = 0.0

current_time = 0.0
previous_time = 0.0


global i,j
i=0
j=0

def callback(msg):
    global vel,z,a,x_deseada
    x_i=msg.data
    x_deseada = x_i.split(delim)
    z=1
    a=a+1

def callback2(msg):
    global ang,k,y_deseada
    y_i=msg.data
    y_deseada = y_i.split(delim)
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


def wrap_to_pi(theta):
    result=np.fmod((theta+ np.pi),(2*np.pi))
    if(result<0):
	result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")  
    wr = 0.0
    wl = 0.0
    rate = rospy.Rate(100)
    delim = ','
    sub = rospy.Subscriber('/help_msg', String, callback)
    sub2 = rospy.Subscriber('/help_msg2', String, callback2)

 
    kp_a=1.0
    kp_d=0.05

    ki_a=0.02
    ki_d=0.02

    #Variable initialisations


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

		x_i3=len(x_deseada) 
	        y_i3=len(y_deseada)

	        current_time = rospy.get_time() 
                previous_time = rospy.get_time()
		print(x_i3)
		
	        r=0

	    # Compute time since last loop
	    rospy.Subscriber('/wr',Float32,wr_callback)
            rospy.Subscriber('/wl',Float32,wl_callback)

	    w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
            pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)
            loop_rate = rospy.Rate(10)


            current_time = rospy.get_time()
            dt = (current_time - previous_time)
            previous_time = current_time
	    
	    
	    pose.theta = pose.theta + dt * m * ((wr - wl) / l)
            pose.x += dt * m * ((wr + wl) / 2) * np.cos(pose.theta)
            pose.y += dt * m * ((wr + wl) / 2) * np.sin(pose.theta)
	    
	    wrap_to_pi(pose.theta)
	    
	    error_a = (math.atan2(float(y_deseada[q]),float(x_deseada[q])))-pose.theta
	    error_d =np.sqrt(np.square(float(x_deseada[q])-pose.x) + np.square(float(y_deseada[q])-pose.y))

	    error_acumulado_a += error_a * dt
	    error_acumulado_d += error_d * dt
	    
	    accion_proporcional_a = kp_a * error_a
	    accion_proporcional_d = kp_d * error_d

	    accion_integral_a = ki_a * error_acumulado_a
	    accion_integral_d = ki_d * error_acumulado_d

	    control_a= accion_proporcional_a + accion_integral_a
	    control_d= accion_proporcional_d + accion_integral_d
            
            velocidad_a = ultima_medicion_a + ((control_a - ultima_medicion_a))
	    velocidad_d = ultima_medicion_d + ((control_d - ultima_medicion_d))
	    ultima_medicion_a = velocidad_a
	    ultima_medicion_d = velocidad_d
	    
	    
	    wr = 0
            wl = 0
	  
	    msg.linear.x = velocidad_d
	    msg.angular.z = velocidad_a

		

	    print_info = "%3f | %3f | %3f | %3f " %(error_d,error_a,velocidad_d,q)
            rospy.loginfo(print_info)
	
	    if error_d < 0.15 :
	    	msg.linear.x = 0.0
            	msg.angular.z = 0.0
            	print("Motion Completed")
         
	    	q=q+1
		pose.x=0.0
		#pose.z=0.0
	        pose.y=0.0
	        pose.theta=0.0
		
                loop_rate.sleep()
		#rospy.signal_shutdown("Square Completed")


            w_pub.publish(msg)
	    pose_pub.publish(pose)
            rate.sleep()           
