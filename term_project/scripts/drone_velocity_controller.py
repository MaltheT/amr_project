#!/usr/bin/env python

#imports for robot controlles
import rospy
import csv 
import math
import numpy as np
from geometry_msgs.msg import TwistStamped, Pose
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool


def quaternion_to_euler(x, y, z, w):        
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))        
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))        
    X = math.atan2(t0, t1)
    Y = math.asin(t2)
    Z = math.atan2(t3, t4)        
    return X, Y, Z


class DroneController:
    def __init__(self):
        rospy.init_node('drone_velocity_controller') 
    
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.yaw = 0.

        self.goal_subscriber = rospy.Subscriber('/goal', Pose, self.goal_callback)
        self.pose_subscriber = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_callback)
        self.velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        rospy.Timer(rospy.Duration(120), self.writeToCsv, oneshot=True)

        # initial goal for hovering
        self.x_goal = 0.
        self.y_goal = 0.
        self.z_goal = 1.
        self.yaw_goal = 0.

        #variables describing error, previous error and accumulated error
        self.e_x = 0.
        self.e_y = 0.
        self.e_z = 0.
        self.e_yaw = 0.

        self.e_x_prev = 0.
        self.e_y_prev = 0.
        self.e_z_prev = 0.
        self.e_yaw_prev = 0.

        self.e_x_acc = 0.
        self.e_y_acc = 0.
        self.e_z_acc = 0.
        self.e_yaw_acc = 0.

        #gains for linear velocities
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.1
        
        #gains for yaw angle
        self.kp_ang = 0.75
        self.ki_ang = 0.01
        self.kd_ang = 0.5

        #logging for plotting
        self.x_log = []
        self.y_log = []
        self.z_log = []
        self.dist_log = []
        self.yaw_log = []
        self.desired_angle_log = [] 

        # putting control callback on timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)

        # Output a few position control setpoint so px4 allow to change into "Offboard" flight mode (i.e. to allow control code running from a companion computer)
        # Note: this can be any control setpoint (e.g. velocities, attitude , etc.)
        r = rospy.Rate(50)
        for i in range (110):
            self.PID()
            r.sleep()

        #initialize the drone
        self.initialize_drone()



    """
    Callbacks for controller, position and goal 
    """
    def control_callback(self, event):
        #print ('Timer called at ' + str(event.current_real))  
        self.PID()


    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        _, _, self.yaw = quaternion_to_euler(data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w)

        #updating the log
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.z_log.append(self.z)
        self.yaw_log.append(self.yaw)

        #calculating the distance 
        dist = np.linalg.norm(np.array([self.x, self.y, self.z]) - np.array([self.x_goal, self.y_goal, self.z_goal]))
        self.dist_log.append(dist)


    def goal_callback(self, data):
        self.x_goal = data.position.x
        self.y_goal = data.position.y
        self.z_goal = data.position.z


    """
    PID Controller 
    """
    def PID(self):
        #current error
        self.e_x = self.x_goal - self.x
        self.e_y = self.y_goal - self.y
        self.e_z = self.z_goal - self.z

        # only accumilate error if the drone is in the air
        if (self.z > 0.5):
            self.e_x_acc += self.e_x
            self.e_y_acc += self.e_y
            self.e_z_acc += self.e_z
            self.e_yaw_acc += self.e_yaw
        else:
            self.e_x_acc += 0
            self.e_y_acc += 0
            self.e_z_acc += 0
            self.e_yaw_acc += 0


        #calculating the new linear velocities in each direction
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = self.kp * self.e_x + self.ki * self.e_x_acc + self.kd * (self.e_x - self.e_x_prev)
        msg.twist.linear.y = self.kp * self.e_y + self.ki * self.e_y_acc + self.kd * (self.e_y - self.e_y_prev)
        msg.twist.linear.z = self.kp * self.e_z + self.ki * self.e_z_acc + self.kd * (self.e_z - self.e_z_prev)

        #calculation of the desired angle 
        desired_angle = math.atan2(self.x_goal - self.x, self.y_goal - self.y)
        self.e_yaw = -self.yaw + desired_angle

        #updating the log 
        self.desired_angle_log.append(desired_angle)

        #correcting the yaw in case of switching between +- pi   
        if(self.e_yaw < -math.pi):
            self.e_yaw += 2 * math.pi
        
        elif(self.e_yaw > math.pi):
            self.e_yaw -= 2 * math.pi

        #calculating new yaw angle 
        #msg.twist.angular.z = self.kp_ang * self.e_yaw + self.ki_ang * self.e_yaw_acc + self.kd_ang * (self.e_yaw - self.e_yaw_prev)
        
        #updating previous error
        self.e_x_prev = self.e_x
        self.e_y_prev = self.e_y
        self.e_z_prev = self.e_z
        self.e_yaw_prev = self.e_yaw

        #publishing the new values 
        self.velocity_pub.publish(msg)


    """
    Initialization of the drone 
    """
    def initialize_drone(self):

        # Set Mode
        print ("\nSetting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = change_mode(custom_mode="OFFBOARD")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)

        # Arm
        print ("\nArming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" %e)


    """
    Data for plotting
    """
    def writeToCsv(self, event):
        with open('x.csv', 'w') as file:
            writer = csv.writer(file) 
            writer.writerow(self.x_log)
            file.close()
        print("x written to csv")

        with open('y.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.y_log)
            file.close()
        print("y written to csv")    

        with open('z.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.z_log)
            file.close()
        print("z written to csv")

        with open('distance.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.dist_log)
            file.close()
        print("distance written to csv")

        with open('yaw.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.yaw_log)
            file.close()
        print("yaw written to csv")

        with open('desired_angle.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.desired_angle_log)
            file.close()
        print("desired angle written to csv")


if __name__ == '__main__':
    try:
        DroneController()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")