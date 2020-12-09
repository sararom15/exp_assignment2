#!/usr/bin/env python

## @file state_machine.py 
# @brief This node implements a state machine 
# 
# Details: It is the main node: it receives commands from the commander node, implements the state machine and sends the target positions to display node. 
# 

#ros + python library 
import rospy
import roslib
import time
import math
import random

#state machine 
import smach 
import smach_ros

#message ros 
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String, Float64 #commands 
from tf import transformations 
from gazebo_msgs.msg import LinkState 
from nav_msgs.msg import Odometry 

#import numpy and scipy 
import numpy as np 
from scipy.ndimage import filters 

import imutils

#Open CV 
import cv2 
from sensor_msgs.msg import CompressedImage 

#actionlib
import actionlib 
import actionlib.msg 
import exp_assignment2.msg 

VERBOSE = False

vel = Twist() 
angle_camera = Float64() 
vel_stop = Twist() 


#random choice 
def user_action(): 
    return random.choice(['play', 'sleep'])




#client of actionlib server for the dog robot which moves randomly in Normal state
def move_dog_randomly(): 
    #Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

     # Waits until the action serve has started up and started listening for goal 
    client.wait_for_server() 

    #Create a goal to sent to action server 
    goal = exp_assignment2.msg.PlanningGoal() 
    goal.target_pose.pose.position.x = random.randrange(-8,8,1)  
    goal.target_pose.pose.position.y = random.randrange(-8,8,1)
    goal.target_pose.pose.position.z = 0

    #Send the goal to the action server 
    client.send_goal(goal) 

    #waits for the serve to finish performing the action. 
    client.wait_for_result() 

    return client.get_result() 





#client of actionlib serve for the dog robot which goes to sleep 
def move_dog_home(): 
    #Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

     # Waits until the action serve has started up and started listening for goal 
    client.wait_for_server() 

    #Create a goal to sent to action server 
    home = exp_assignment2.msg.PlanningGoal() 
    home.target_pose.pose.position.x = 7
    home.target_pose.pose.position.y = 7
    home.target_pose.pose.position.z = 0

    #Send the goal to the action server 
    client.send_goal(home) 

    #waits for the serve to finish performing the action. 
    client.wait_for_result() 

    return client.get_result() 

#detection ball
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        #rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid

            
            
            

            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                rospy.set_param('state',1)
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = 0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100) 
                self.vel_pub.publish(vel)

                
            


        else:

            ## set parameter service state = 1 
            rospy.set_param('state',2)
            vel = Twist()
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)


        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)                                 


def vel_clbk(dog_vel): 
    vel.linear.x = dog_vel.linear.x
 


## Normal state definition    
class Normal(smach.State):

    ## inizialization
    def __init__(self):
        ## 2 outcomes defined 
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.command = String() 
        
        
    ## execution 
    def execute(self, userdata):
        ## set parameter service state = 1 
        
        
        ## Main Loop 
        while True: 
            #move randomly 
            move_dog_randomly()
            time.sleep(2) 
            


            self.command = user_action() 
            rospy.loginfo('command received is %s', self.command) 

            if self.command == "sleep" :
                return 'outcome1'

            if self.command == "play" : 
                #look for a ball 
                ic = image_feature() 
            
                #get state 
                state_ = rospy.get_param('state') 
                if state_ == 1: #detected ball 
                    return 'outcome2'
                if state_ == 2: #no detected ball 
                        time.sleep(2) 


                    

            




## Sleep State definition 
class Sleep(smach.State):
    ## Initialization
    def __init__(self):
        ## 1 outcome defined : Normal 
        smach.State.__init__(self, outcomes=['outcome1'])
        
    
    ## Execution 
    def execute(self, userdata):
        move_dog_home()

        ## sleep for a while 
        time.sleep(14) 
         
        return 'outcome1'



## Play state definition 
class Play(smach.State): 
    ## initialization 
    def __init__(self): 
        ## 1 outcome defined : Normal 
        smach.State.__init__(self, outcomes=['outcome2'])


    ## Execution
    def execute(self, userdata): 
        
        time.sleep(15) 
        rospy.Subscriber("/robot/cmd_vel", Twist, vel_clbk) 
        angle_pub = rospy.Publisher("/robot/joint_position_controller/command",
                                    Float64, queue_size=1)    

        stop_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size = 1)
        
        while vel.linear.x > 0.01: 
            time.sleep(1) 
        if vel.linear.x < 0.01: 
        
            vel_stop.angular.z = 0
            vel_stop.linear.x = 0
            stop_pub.publish(vel_stop)

            angle_camera.data = 0.0
            while angle_camera.data < 6.27: 
                angle_camera.data = angle_camera.data + 0.1 
                angle_pub.publish(angle_camera)
                time.sleep(2) 

        #return to a Normal state 
        return 'outcome2'

## Main Function definition 
def main():
    ## init the ros node 
    rospy.init_node('state_machine')
  


    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'outcome1':'SLEEP',
                                            'outcome2':'PLAY'})
        
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'outcome1':'NORMAL'})
        
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'outcome2':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


