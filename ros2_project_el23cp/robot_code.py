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

import threading
import sys, time
import cv2
import numpy as np
#import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from rclpy.exceptions import ROSInterruptException
#import signal

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

        for _ in range(100):  # Stop for a brief momentcolcon build  && source ~/.bashrc
            self.publisher.publish(desired_velocity)
            self.rategreen_found.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def rotate(self):
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
        self.goal_done = False

    def send_goal(self, x, y, yaw):
        self.goal_done = False
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
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  
    
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_done = True

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

'''
___________ COLOURS ___________
'''    
class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        # Initialise any flags that signal a colour has been detected (default to false)

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.find_colour, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        self.green_found = False
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.too_close = False
        
        



    def find_colour(self, data):


        # Convert the received image into a opencv image
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)

        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the colour you wish to identify - green
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([180 + self.sensitivity, 255, 255])
        
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rg_mask, blue_mask)
        
        filtered_img = cv2.bitwise_and(image, image, mask=rgb_mask)
        #green_img = cv2.bitwise_and(image, image, mask=green_mask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        
        # Blue
        contours, hierarchy = cv2.findContours(blue_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        self.blue_found = False

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)

            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)

            if cv2.contourArea(c) > x: #<What do you think is a suitable area?>

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                x, y, w, h = cv2.boundingRect(c)

                colour = (0,0,0)
                thickness = 3

                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)

                # Then alter the values of any flags

                self.blue_found = True
                self.count +1

                self.get_logger().info('Blue is found')
            else:
                self.blue_found = False   

        #GREEN
        contours, hierarchy = cv2.findContours(green_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        self.green_found = False

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > x: #<What do you think is a suitable area?>

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                x, y, w, h = cv2.boundingRect(c)

                colour = (0,0,0)
                thickness = 3

                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)

                # Then alter the values of any flags

                self.green_found = True
                self.count +1
                self.get_logger().info('Green is found')

            else:
                self.green_found = False   

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        
        #GREEN
        contours, hierarchy = cv2.findContours(green_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        self.red_found = False

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > x: #<What do you think is a suitable area?>

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                x, y, w, h = cv2.boundingRect(c)

                colour = (0,0,0)
                thickness = 3

                start_pt = (x,y)
                end_pt = (x+w, y+h)
                cv2.rectangle(image, start_pt, end_pt, colour, thickness)
                
                self.red_found = True
                self.count +1
                self.get_logger().info('Red is found')
            else:
                self.red_found = False   


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.go_to_pose = GoToPose()
        self.motion = Motion()
        self.colourIdentifier = colourIdentifier()

        self.state = "go to corner"

        self.current_x = None
        self.current_y = None
        self.blue_found = False
        self.red_found = False
        self.green_found = False

        self.count = 0

        self.create_timer(0.1, self.robot_check)
        
#    def go_to_corner(self, x, y, yaw):
#        self.go_to_pose.send_goal(x, y, yaw)# Defining main()

    def go_to_blue(self, x, y, yaw):
        # use cv.moment to make it go closer to blue contour
        # rotate until find blue
        self.go_to_pose.send_goal(x, y, yaw)
        return

    #def checking_colours(self):
        #self.get_logger().info('Looking for colours')
        #self.colourIdentifier.find_colour()
          
    def robot_check(self):
        if self.state == "go to corner":
            self.get_logger().info('Starting Position')
            self.x_val = -9.8
            self.y_val = -15.0
            self.go_to_pose.send_goal(self.x_val, self.y_val, 0.0024)
            self.state = "scanning"
            
        
        elif self.state == "scanning":
            # loop through block flags, as it rotates
            #   recording locations
            # rotate by small degree until certain count
            while self.count < 3:
                self.get_logger().info('Scanning')
                
                self.motion.rotate()
                self.colourIdentifier.find_colour()
                
                return

            if self.count == 3 & self.blue_found == True:
                self.get_logger().info('All colours found')
                self.state = "go to blue"
                self.go_to_pose.send_goal(-4.7, -11.5, 0.0024)
                return
            
        elif self.state == "go to blue":
            #get location when it last saw blue
            # position x
            # position y
            # self.go_to_pose.send_goal(position x, position y, 0.0024)
            return
        elif self.state == "stop":
            self.motion.stop()
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
