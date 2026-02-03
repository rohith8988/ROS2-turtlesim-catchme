#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn
from std_msgs.msg import String 

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('spawn_random_turtles')
        
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn not available, waiting...')

        self.declare_parameter("name_counter", 1)
        self.name_counter = int(self.get_parameter("name_counter").value)
        
        self.target_caught_sub = self.create_subscription(String, '/target_caught', self.on_target_caught, 10)
        self.current_target_pub = self.create_publisher(String , "/current_target", 10)

        self.current_target = None 
        
        # Flag to trigger spawning
        self.spawn_needed = True 
        
        # Timer to check for spawn requests instead of a loop in main
        self.timer = self.create_timer(0.5, self.spawn_loop_tick)

    def spawn_loop_tick(self):
        if self.spawn_needed:
            self.spawn_needed = False
            self.spawn_turtle()

    def spawn_turtle(self):
        name = f'turtle_{self.name_counter}'
        x = random.uniform(1.0, 10.0) 
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 6.28)

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        # Use async call with a callback instead of spin_until_future_complete
        future = self.client.call_async(request)
        future.add_done_callback(lambda future: self.on_spawn_done(future, x, y, theta))

    def on_spawn_done(self, future, x, y, theta):


        try:
            response = future.result()
            self.current_target = response.name
            msg = String()
            msg.data = self.current_target
            self.current_target_pub.publish(msg)

            self.get_logger().info(f'Spawned turtle: {response.name} at ({x:.2f}, {y:.2f})')
            self.name_counter += 1
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def on_target_caught(self, msg):
        self.get_logger().info(f'Target caught: {msg.data}')
        self.spawn_needed = True

def main(args=None):    
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    
    try:
        # spin() now handles everything via timers and subscriptions
        rclpy.spin(turtle_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        turtle_spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()