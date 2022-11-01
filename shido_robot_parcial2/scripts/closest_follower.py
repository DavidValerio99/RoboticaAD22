#!/usr/bin/env python  

from cmath import isfinite
import rospy  
import numpy as np

from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 

# This class will subscribe to the /base_scan topic and print some data 

class ClosestObjectClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  
        rospy.Subscriber("scan", LaserScan, self.lidar_cb)  

        ############ CONSTANTS AND VARIABLES ################  

        self.flag = 0 
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

        #********** INIT NODE **********###  
        self.movement = Twist() 


        r = rospy.Rate(10) #10Hz  

        print("Node initialized 10hz") 

        while not rospy.is_shutdown():  
            if self.flag == 1:
                #Get array of distances of object near
                self.closest_object = min(self.lidar.ranges)
                #Confirm the object is close enough "not infinte distance"
                if np.isfinite(self.closest_object):
                    print("The distance to the closest object is: " + str(self.closest_object))
                    self.index_closest_object = self.lidar.ranges.index(self.closest_object)
                    self.angle_closest = self.lidar.angle_min + self.index_closest_object * self.lidar.angle_increment
                    print("The angle to the closest object is: " + str(self.angle_closest))
                    self.move()
                    self.move_pub.publish(self.movement) 
                else:
                    #No object is detected
                    self.movement.linear.x = 0
                    self.movement.angular.z = 0
                    #Command to stop the robot before finishing the node
                    self.move_pub.publish(self.movement)
                    print("No object is detected")

            r.sleep()
    

    def lidar_cb(self, msg_l):  
        ## This function receives the lidar message and copies this message to a member of the class  
        #print("lidar callback")
        self.lidar = msg_l   
        self.flag = 1  

    def move(self):
        #Movement based on lidar measurements calculated

        #If the angle is greater than 0.2, the robot will turn towards it
        if(abs(self.angle_closest) > 0.2):
            print("anglemovement:"+str(self.angle_closest))
            print("turning")
            #Linear velocity = 0, just turn towards the closes object
            self.movement.linear.x  = 0.0
            #closest angle determines the direction of the turn (+:CW, -:CCW)
            self.movement.angular.z = -0.7*self.angle_closest
        #Once the robot has turned towards the closest object, the robot will start 
        #moving towards it only if its distance is greater than the detection zone
        elif(self.closest_object > 0.5): #Detection zone is 50 cm from lidar: ~30 cm 
                                        #to the robot edge + 20 cm of safety range
            print("linear movement: " + str(self.closest_object))
            print("angle movement:" + str(self.angle_closest))
            print("moving")
            #Small angular velocity to take into account the error
            self.movement.angular.z = 0.0*self.angle_closest
            #Linear velocity greater to move faster towards the object
            self.movement.linear.x  = 0.35*self.closest_object
            
        #Once the object is within the detection zone the robot will stop
        elif(self.closest_object < 0.5):
            if(self.angle_closest >0.2):
                #Linear velocity = 0, just turn towards the closest object
                self.movement.linear.x  = 0.0
                #closest angle determines the direction of the turn (+:CW, -:CCW)
                self.movement.angular.z = 0.5*self.angle_closest
        else:
            print("stop")
            self.movement.linear.x  = 0.0
            self.movement.angular.z = 0.0

    def cleanup(self):  
        #This function is called just before finishing the node  
        self.movement.linear.x  = 0.0
        self.movement.angular.z = 0.0
        #Command to stop the robot before finishing the node
        self.move_pub.publish(self.movement) 
        print("I'm dying, bye bye!!!")  


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("closest_object_detector", anonymous=True)  

    ClosestObjectClass()
