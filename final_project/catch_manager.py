#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtlesim.srv import Kill


class CatchManager(Node):
    def __init__(self):
        super().__init__('catch_manager')

        self.target_name = None
        self.main_pose = None
        self.target_pose = None
        self.kill_in_progress = False

        self.subscriber_target_turtle = None


        self.kill_client = self.create_client(Kill, '/kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service /kill is not available yet, waiting.....")

        self.subscriber_current_target = self.create_subscription(String,'/current_target',self.on_new_target, 10 )
        self.subscriber_main_pose = self.create_subscription(Pose,'/turtle1/pose',self.on_main_pose,10 )

        self.target_caught_pub = self.create_publisher(String, "/target_caught", 10)

        
        
        self.catch_threshold = 0.5
        self.timer_ = self.create_timer(0.05, self.check_catch)

    def on_main_pose(self,msg):
        self.main_pose = msg
        

    def on_target_pose(self,msg):
        self.target_pose = msg

    def on_new_target(self,msg):
        new_name = msg.data
        if not new_name or new_name == self.target_name:
            return

        self.target_name = new_name
        self.target_pose = None
        self.kill_in_progress = False

        if self.subscriber_target_turtle is not None:
            self.destroy_subscription(self.subscriber_target_turtle)
            self.subscriber_target_turtle = None

        self.subscriber_target_turtle = self.create_subscription(Pose, "/" + self.target_name +"/pose", self.on_target_pose,10)
        self.get_logger().info(f'Tracking target: /{self.target_name}/pose')


    def check_catch(self):
        if self.kill_in_progress == True:
            return
        if self.main_pose is None or self.target_pose is None or self.target_name is None:
            return 

        #compute the distance between the main turtle and the target turtle
        dx = self.main_pose.x - self.target_pose.x
        dy = self.main_pose.y - self.target_pose.y

        dist = math.sqrt(dx**2 + dy**2)

        
        if dist <= self.catch_threshold:
            self.kill_in_progress = True
            self.kill_target(self.target_name)



    def kill_target(self,kill_name):
        request = Kill.Request()
        request.name = kill_name


        future = self.kill_client.call_async(request)
        future.add_done_callback(lambda f: self.on_kill_done(f, kill_name))



    def on_kill_done(self, future , name):

        try:
            future.result()

            msg = String()
            msg.data = name
            self.target_caught_pub.publish(msg)


            if self.subscriber_target_turtle is not None:
                self.destroy_subscription(self.subscriber_target_turtle)
                self.subscriber_target_turtle = None

            self.target_name = None
            self.target_pose = None
            

        except Exception as e:
            self.get_logger().error(f"error at on_kill_done for {name}: {e}")
            pass

        finally:
            self.kill_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = CatchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

