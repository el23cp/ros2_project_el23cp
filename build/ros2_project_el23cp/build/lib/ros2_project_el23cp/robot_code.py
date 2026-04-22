'''
load world: 
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py

load map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_el23cp/map/map.yaml

to launch new/updated code: ALWAYS colcon build  && source ~/.bashrc

ros2 run ros2_project_el23cp robot_code

to project directory: cd  ~/ros2_ws/src/ros2_project_el23cp
'''

#FORWARD THEN SCANNING imports
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos
import threading
import sys, time
import cv2
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Motion:
    def __init__(self, node):
        #super().__init__('firstwalker')
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = node.create_rate(10)  # 10 Hz

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.5  # Forward with 0.2 m/s

        #for _ in range(100):  # Stop for a brief momentcolcon build  && source ~/.bashrc
        self.publisher.publish(desired_velocity)
            #self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def rotate(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  
        desired_velocity.angular.z = 0.628  # Rotate at 0.2 deg
        
        self.publisher.publish(desired_velocity)
        #self.rate.sleep()
    
    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0 # Stop rotating     
        #for _ in range(50):  # Stop for a brief moment
        self.publisher.publish(desired_velocity)
        #    self.rate.sleep()

'''
___________ GOING_TO_POSITIONS ___________
'''
class GoToPose:
    def __init__(self, node):
        #super().__init__('navigation_goal_action_client')
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.goal_done = False

    def send_goal(self, x, y, yaw):
        self.goal_done = False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

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
            self.node.get_logger().info('Goal rejected')
            self.goal_done = True
            return

        self.node.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  
        result = future.result().result
        self.node.get_logger().info(f'Navigation result: {result}')
        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.

'''
___________ COLOURS ___________
'''    
class colourIdentifier:
    def __init__(self, node):
        #super().__init__('cI')

        self.node = node
        self.bridge = CvBridge()
        self.subscription = node.create_subscription(Image, '/camera/image_raw', self.find_colour, 10)
        #self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        
        self.green_found = False
        self.blue_found = False
        self.red_found = False

        self.count = 0
        
        #self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = node.create_rate(10)  # 10 Hz
        #self.too_close = False

    def find_colour(self, data):


        # Convert the received image into a opencv image
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Set the upper and lower bounds for the colour you wish to identify - green
        hsv_red_lower1 = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper1 = np.array([0 + self.sensitivity, 255, 255])
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180 + self.sensitivity, 255, 255])
               
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)        
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)   
        red_mask = red_mask1 | red_mask2
        
        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rg_mask, blue_mask)
        
        filtered_img = cv2.bitwise_and(image, image, mask=rgb_mask)

        # Blue
        contours, hierarchy = cv2.findContours(blue_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        
        if len(contours) > 0:
            
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M['m00'] == 0:
                return
            
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            if cv2.contourArea(c) > 500:
                
                x, y, w, h = cv2.boundingRect(c)

                self.blue_cx = cx
                self.image_width = image.shape[1]
                
                colour = (255,0,0)
                thickness = 3
                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)

                if not self.blue_found:
                    self.node.get_logger().info('Blue is found')
                    self.blue_found = True
                
                self.blue_area = cv2.contourArea(c)

        #GREEN
        contours, hierarchy = cv2.findContours(green_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        if len(contours) > 0:
            
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>

                x, y, w, h = cv2.boundingRect(c)
                colour = (120,0,120)
                thickness = 3
                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)

                if not self.green_found:
                    self.node.get_logger().info('Green is found')
                    self.green_found = True  

        #RED
        contours, hierarchy = cv2.findContours(red_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        if len(contours) > 0:
            
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>

                x, y, w, h = cv2.boundingRect(c)
                colour = (120,0,120)
                thickness = 3
                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)
                
                if not self.red_found:
                    self.node.get_logger().info('Red is found')
                    self.red_found = True                    
                
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.go_to_pose = GoToPose(self)
        self.motion = Motion(self)
        self.colourIdentifier = colourIdentifier(self)

        self.state = "scanning"

        self.blue_found = False
        self.red_found = False
        self.green_found = False

        self.corner_1 = False
        self.corner_2 = False
        self.corner_3 = False        

        self.count = 0
        self.rotation_steps = 0
        self.blue_pose = None

        self.create_timer(0.1, self.robot_check)
        self.sent_corner_goal = False
            
        
    def update_colour_flags(self):
        self.blue_found = self.colourIdentifier.blue_found
        self.green_found = self.colourIdentifier.green_found
        self.red_found = self.colourIdentifier.red_found

        self.count = sum([self.blue_found, self.green_found, self.red_found])
   
    def robot_check(self):
        self.update_colour_flags()
        
        if self.state == "go to corner":
            #self.sent_corner_goal = False

            if not self.sent_corner_goal:
                #if not hasattr(self, "sent_corner_goal"):
                self.get_logger().info('Going to corner')

                self.motion.stop()

                if not self.corner_1:
                    self.x_val = -8.5
                    self.y_val = -13.0
                    self.corner_1 = True
                    
                elif not self.corner_2:
                    self.x_val = 7.0 
                    self.y_val = 12.0 
                    self.corner_2 = True
                    
                elif not self.corner_3:
                    self.x_val = -10.0
                    self.y_val = 3.0 
                    self.corner_3 = True

                else:
                    self.get_logger().info("All corners explored")
                    return
                    
                self.go_to_pose.send_goal(self.x_val, self.y_val, 0.0024)
                self.current_x = self.x_val
                self.current_y = self.y_val
                #self.sent_corner_goal = True
                
            elif self.go_to_pose.goal_done:
                self.sent_corner_goal = False
                self.state = "scanning"
        
        elif self.state == "scanning":
            self.motion.rotate()
            self.rotation_steps += 1
            #self.get_logger().info("Scanning")


            if self.rotation_steps < 50:
                return

            self.motion.stop()
            self.rotation_steps = 0

            # store blue pose when first seen
            if self.blue_found and self.blue_pose is None:
                self.blue_pose = {
                    "x": self.current_x,
                    "y": self.current_y,
                    "yaw": self.rotation_steps * 0.0628  
                }
                self.get_logger().info("Stored blue direction")

            if self.red_found and self.green_found and self.blue_found:
                self.get_logger().info('All colours found')
                self.motion.stop()
                self.state = "go to blue"
                return
            else:
                self.state = "go to corner"
                return
                
            
        elif self.state == "go to blue":

            if not hasattr(self.colourIdentifier, "blue_cx"):
                self.motion.rotate()
                return
            
            cx= self.colourIdentifier.blue_cx
            width = self.colourIdentifier.image_width
            area = self.colourIdentifier.blue_area
            #self.get_logger().info(f'Area: {area}')

            blue_centre = width/2
            error = cx - blue_centre

            twist = Twist()

            if abs(error) > 20:
                twist.angular.z = -0.002*error
            else:
                if area < 315000:
                    print("Not close enough")
                    self.too_close = True
                    twist.linear.x = 0.2
                elif area > 325000:
                    print("Too close")
                    self.too_close = False
                    twist.linear.x = -0.2
                else:
                    print("Reached target. Within 1m")
                    #self.too_close = False
                    self.motion.stop()   
                
            self.motion.publisher.publish(twist)
    
# Defining main()
def main(args=None):
    
    rclpy.init(args=args)    
    explorer = Explorer()
    rclpy.spin(explorer)

if __name__ == '__main__':
    main()
