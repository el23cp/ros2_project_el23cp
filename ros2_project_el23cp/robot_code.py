'''
load world: 
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py

load map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_el23cp/map/map.yaml

to launch new/updated code: ALWAYS colcon build  && source ~/.bashrc

ros2 run ros2_project_el23cp robot_code

to project directory: 
'''

#FORWARD THEN SCANNING imports
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal

#GOING TO POSITION imports
#import rclpy
from rclpy.action import ActionClient
#from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos

'''
___________ FORWARD_THEN_SCANNING ___________
'''


class Motion(Node):
    def __init__(self):
        super().__init__('firstwalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.5  # Forward with 0.2 m/s

        for _ in range(100):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def rotate_on_spot(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Forward with 0.2 m/s
        desired_velocity.angular.z = 0.628  # Rotate at 0.2 deg
        
        for _ in range(100):  # Rotate
            self.publisher.publish(desired_velocity)
            self.rate.sleep()
    
    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0 # Stop rotating     
        for _ in range(50):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

'''
___________ GOING_TO_POSITIONS ___________
'''
class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  
    
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.
        '''
        ## Access the current pose
        current_pose = feedback_msg.feedback.current_pose
        position = current_pose.pose.position
        orientation = current_pose.pose.orientation

        ## Access other feedback fields
        navigation_time = feedback_msg.feedback.navigation_time
        distance_remaining = feedback_msg.feedback.distance_remaining

        ## Paw)rint or process the feedback data
        self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        self.get_logger().info(f'Distance Remaining: {distance_remaining}')
        '''
        
class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.go_to_pose = GoToPose()
        self.motion = Motion()
        self.state = "go_to_corner"
        self.current_x
        self.current_y
        self.direction, self.row()
        self.create_timer(0.1, self.loop)
        
    def go_to_corner(self, x, y, yaw):
        self.go_to_pose.send_goal(x, y, yaw)# Defining main()
    
    def robot_check(self):
        if self.state == "go to corner":
            self.x_val = -9.8
            self.y_val = -15.0
            self.state = "waiting"
            return
        
        if self.state == "waiting":
            
            self.x_val = -9.8
            self.y_val = -15.0
            return
                        
            
# Defining main()
def main(args=None):
    rclpy.init(args=args)
    
    explorer = Explorer()
      
    rclpy.spin(explorer)
    # from go_to_pos
    '''
    rclpy.init(args=args)

    go_to_pose = GoToPose()
    motion = Motion()
    # go to corner: 
    x_val = -9.8
    y_val = 15.0
    start_yaw = 0.0024 
    
    go_to_pose.send_goal(x_val, y_val, start_yaw)  # example coordinates
    rclpy.spin_once(go_to_pose)
    
    
    # move 5m x-way then scan every 5m

    scan = motion.rotate_on_spot()
    
    # checks if it can see any blocks. 
    
    # if all 3, then walk to blue
    
    # if not "find blue/red/green"
    #checks if all 3, then walk to blue
    # if not find remaining colour
    # checks if all 3, if yes, walk to blue, if not walk find last one,
    # find method using grid below
    while x_val < 9.95 & y_val < 6:
        new_x = x_val + 5
        new_y = y_val - 5
        go_to_pose.send_goal(new_x, new_y, start_yaw)
    '''

if __name__ == '__main__':
    main()
