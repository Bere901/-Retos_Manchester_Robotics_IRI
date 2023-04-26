#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import pi

vel=[0.3,0.0,0.3,0.0,0.3,0.0,0.3,0.0]
ang=[0.0,0.5,0.0,0.5,0.0,0.5,0.0,0.5]
global i,j
i=0
j=0

class square:
    def __init__(self):
        #initialise wheel velocity variables
        self.wr = 0.0
        self.wl = 0.0

        #Setup ROS subscribers and publishers
        rospy.Subscriber('/wr',Float32,self.wr_callback)
        rospy.Subscriber('/wl',Float32,self.wl_callback)

        self.w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        #setup node
        rospy.init_node("Square")
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    # Callbacks for wheel velocities and commands
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    # Main function
    def run(self):
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

        # Main Loop
        while not rospy.is_shutdown():
	    global i,j
            # Compute time since last loop
            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time
            # Update distance and angle from the velocity measurements
            distance += 0.05 * (self.wr + self.wl) * 0.5 * dt
            angle += 0.05 * (self.wr - self.wl) / 0.18 * dt
            self.wr = 0
            self.wl = 0
            # state 0 = moving along a straight
            msg.linear.x = vel[i]
            msg.angular.z = ang[i]
	    print(distance)
	    print(angle)
            # If at end of a side
            if distance > 1:
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
  	    if angle > 1.4 and j==1:
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
            	
            

            # Publish message and sleep
            self.w_pub.publish(msg)

            self.rate.sleep()
            
    # Separate stop function for stopping when ROS shuts down
    def stop(self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.w_pub.publish(msg)

if __name__ == "__main__":
    sq = square()

    try:
        sq.run()
    except rospy.ROSInterruptException:
        None
