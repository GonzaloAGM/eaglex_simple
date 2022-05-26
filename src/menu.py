#!/usr/bin/env python

from __future__ import print_function

import rospy
from topic_tools.srv import MuxSelect

class MenuClass():

    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ## SUBSCRIBERS ## 
        #rospy.Subscriber("joy", LaserScan, self.laser_cb) 

    #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        rospy.wait_for_service('mux_cmdvel/select')
        print("Node initialized 1hz")
        print("1: joy_mode  2: auto_mode")
        while not rospy.is_shutdown(): 
            command = input("Enter command:")
            
            if(command=='1'):
                self.mux_client("cmd_vel_joy")
            if(command=='2'):
                self.mux_client("cmd_vel_auto")
            
            r.sleep()     

    def mux_client(self,x):
        #rospy.wait_for_service('mux_cmdvel/select')
        try:
            mux_call = rospy.ServiceProxy('mux_cmdvel/select', MuxSelect)
            resp1 = mux_call(x)
            #print(x)
            return resp1.prev_topic
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    #def select_joy(self):
    #   self.mux_client("cmd_vel_joy")

    #def select_auto(self):
    #   self.mux_client("cmd_vel_auto")

    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.  
        #stop_twist = Twist() 
        #self.pub_cmdvel.publish(stop_twist) 
        pass

if __name__ == "__main__": 
    rospy.init_node("menu", anonymous=True) 
    MenuClass() 
