#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node 
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class MainTurtleNode(Node):
    def __init__(self):
        super().__init__('main_turtle')
        self.target_name = None
        self.main_pose = None
        self.target_pose = None

        self.subscriber_target_turtle = None
        
        #Subscribers
        self.main_pose_sub = self.create_subscription( Pose,'/turtle1/pose',self.on_main_pose,10)
        self.current_target_sub = self.create_subscription(String, '/current_target',self.on_new_target,10)

        #Publishers
        self.cmd_vel_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        
        #variables
        self.catch_threshold = 0.5 
        self.angle_tolerance = 0.2
        self.slow_radius = 1.2
        self.k_angular = 4.0
        self.max_angular = 2.0
        self.k_linear = 1.0
        self.max_linear = 1.5
        
        self.timer_ = self.create_timer(0.05, self.control_tik)


    def on_main_pose(self,msg):
        self.main_pose = msg
        

    def on_target_pose(self,msg):
        self.target_pose = msg    


    def on_new_target(self,msg): #dynamically subscribing and getting the pose of the spawned turtle
        new_name = msg.data
        if not new_name or new_name == self.target_name:
            return

        self.target_name = new_name
        self.target_pose = None

        if self.subscriber_target_turtle is not None:
            self.destroy_subscription(self.subscriber_target_turtle)
            self.subscriber_target_turtle = None

        self.subscriber_target_turtle = self.create_subscription(Pose, "/" + self.target_name +"/pose", self.on_target_pose,10)
        self.get_logger().info(f'Tracking target: /{self.target_name}/pose')


    def control_tik(self):
        if self.target_name == None:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        if self.main_pose is None or self.target_pose is None:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            return


        dx = self.target_pose.x - self.main_pose.x
        dy = self.target_pose.y - self.main_pose.y

        distance = math.sqrt(dx**2 + dy**2)
        
        desired_heading = math.atan2(dy,dx)
        heading_error = desired_heading -self.main_pose.theta
        heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi


        if distance <= self.catch_threshold:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        if abs(heading_error) > self.angle_tolerance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.k_angular* heading_error))
            self.cmd_vel_pub.publish(cmd)
            return
        

        cmd = Twist()
        if distance>self.slow_radius:
            linear = self.k_linear * distance

        else:
            linear = self.k_linear* distance* (distance/self.slow_radius)

        cmd.linear.x = max(0.0,min(self.max_linear, linear))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.k_angular*heading_error))

        self.cmd_vel_pub.publish(cmd)
        return



def main(args = None):

    rclpy.init(args=args)
    node = MainTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()