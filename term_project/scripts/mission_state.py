
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from rrt import RRT
from mavros_msgs.srv import SetMode



class MissionStateMachine:
    #instantiater
    def __init__(self):
        # Creates a node with subscriber
        rospy.init_node('mission_state_machine')
        self.x = 0.
        self.y = 0.
        self.z = 0. 
        self.threshold = 0.2
        size = 2 #size of the obstacle (estimate)
        

        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.goal_publisher = rospy.Publisher('/goal', Pose, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
       
        #lists of the waypoints and obstacles 
        self.obstacles = [[3.25, -1.137, size],[4, 3, size], [-3, 2.9, size]]
        #self.waypoints = [[0, 0, 0],[-3, 0, 2.5],[0, 0, 2]]
        self.waypoints = [[0, 0, 0],[-3, 0, 2.5], [-3, 4.5, 2.5], [3, 4, 2], [3, -4, 2], [0, 0, 2]]
        
        #iterators
        self.curr_waypoint_idx = 1
        self.path_itr = 0

        #getting the whole path through the waypoints  
        self.path = []
        self.find_path()


    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        
        #publishing one point the path 
        path_pose = Pose()            
        path_pose.position.x = self.path[self.path_itr][0]
        path_pose.position.y = self.path[self.path_itr][1]
        path_pose.position.z = self.path[self.path_itr][2]
        self.goal_publisher.publish(path_pose)  
        
        #if x y z values are within the threshold area
        if  ((abs(self.x - self.path[self.path_itr][0]) < self.threshold) and 
            (abs(self.y - self.path[self.path_itr][1]) < self.threshold) and 
            (abs(self.z - self.path[self.path_itr][2]) < self.threshold) and 
            (self.path_itr < len(self.path))):
            self.path_itr += 1 
        

        #for landing 
        if(self.path_itr == len(self.path) - 1):
            print ("\nSetting Mode LAND")
            rospy.wait_for_service('/mavros/set_mode')
            
            try:
                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="AUTO.LAND")
                rospy.loginfo(response)

            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
    

    def find_path(self):
        print("Start RRT path planning")
        for i in range (len(self.waypoints) - 1):
            rrt = RRT(start=[self.waypoints[i][0], self.waypoints[i][1]], goal=[self.waypoints[i + 1][0], self.waypoints[i + 1][1]],
            randArea=[-8, 8], obstacleList=self.obstacles)

            #get the path and reversing it since the output of the function is the reversed path 
            sub_path = rrt.planning()
            sub_path.reverse()

            #adding the z value into the height since 2D rrt algorithm is used
            for point in sub_path:
                point.append(self.waypoints[self.curr_waypoint_idx][2])      
            
            self.curr_waypoint_idx += 1 

            #adding the subpath into the main path 
            self.path.extend(sub_path)



if __name__ == '__main__':
    try:
        MissionStateMachine()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")

