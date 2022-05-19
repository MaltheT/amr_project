from multiprocessing.connection import Listener
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import timeit
import geometry_msgs.msg
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
from rrt import RRT 
import numpy as np
import os, cv2
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool



#class that has robots controller
class MissionStateMachine:
    #instantiater
    def __init__(self):
        # Creates a node with subscriber
        rospy.init_node('mission_state_machine')
        self.x = 0.
        self.y = 0.
        self.z = 0. 
        self.threshold = 0.2

        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.goal_publisher = rospy.Publisher('goal', Pose, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        size = 2
        self.obstacles = [[3.25, -1.137, size],[4, 3, size], [-3, 2.9, size]]
        #self.waypoints = [[-3, 0, 2.5],[0, 0, 2], [0, 0, 0]]
        #self.waypoints = [[-3, 0, 2.5], [-3.0, 4.5, 2.5], [3, 4, 2], [3, -4, 2],[0, 0, 2], [0, 0, 0]]
        #self.waypoints = [[3.0, 0.0, 1.0]] 
        
        self.curr_waypoint_idx = 0
        self.path_iterator = 0 

        #initialize path 
        print("Start RRT path planning")       
        self.rrt = RRT(start=[self.x, self.y], goal=[self.waypoints[self.curr_waypoint_idx][0], self.waypoints[self.curr_waypoint_idx][1]],
            randArea=[-10, 10], obstacleList=self.obstacles)
        self.path = self.rrt.Planning(animation=False)

        for point in self.path:
            point.append(self.waypoints[self.curr_waypoint_idx][2])

        self.path.reverse()


    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        try:
            path_pose = Pose()
            path_pose.position.x = self.path[self.path_iterator][0]
            path_pose.position.y = self.path[self.path_iterator][1]
            path_pose.position.z = self.path[self.path_iterator][2]
            self.goal_publisher.publish(path_pose)

            
        
            #if x y z values are within the threshold area
            if  (self.path_iterator < len(self.path)-1 and
                self.curr_waypoint_idx < len(self.waypoints)-1 and
                (abs(self.x - self.path[self.path_iterator][0]) < self.threshold) and 
                (abs(self.y - self.path[self.path_iterator][1]) < self.threshold) and 
                (abs(self.z - self.path[self.path_iterator][2]) < self.threshold)): 

                #updating the index
                self.path_iterator += 1
                print(round(self.path[self.path_iterator][0],1), round(self.path[self.path_iterator][1],1), round(self.path[self.path_iterator][2],1))

            elif (self.path_iterator == len(self.path)-1):
                self.curr_waypoint_idx += 1
                self.path_iterator = 0 
                self.path = []

                rrt = RRT(start=[self.x, self.y], goal=[self.waypoints[self.curr_waypoint_idx][0], self.waypoints[self.curr_waypoint_idx][1]],
                randArea=[-10, 10], obstacleList=self.obstacles)
                self.path = rrt.Planning(animation=False)

                for point in self.path:
                    point.append(self.waypoints[self.curr_waypoint_idx][2])
                
                self.path.reverse()
        except:
            print("no path yet")
            pass

        
        if(self.curr_waypoint_idx == len(self.waypoints)-1):
            print ("\nSetting Mode LAND")
            rospy.wait_for_service('/mavros/set_mode')
            try:
                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="AUTO.LAND")
                rospy.loginfo(response)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)


if __name__ == '__main__':
    try:
        MissionStateMachine()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")


        
        
        
