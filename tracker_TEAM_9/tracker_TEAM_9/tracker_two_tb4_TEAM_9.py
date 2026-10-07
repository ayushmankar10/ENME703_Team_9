#!/usr/bin/env python3

import math
import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose 

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

from .... import ...  # To put your custom msg package and file



'''
-   This node should be launch from your own Package called "tracker_TEAM_X" (X is your team number.)
-   Your launch file should accept 3 arguments:
        - chaser            # e.g., tb402
        - enemy             # e.g., tb407
        - global_frame      # map
-   launch file naming format: tracker_two_tb4_TEAM_X.launch.py
-   Python file naming format: tracker_two_tb4_TEAM_X.py
'''

'''
For your custom message, follow the naming and data type as follow.
Note: *** you should not forget the message header. ***

Custom message name: TrackerReport.msg

-Data Type                       Name                Comment                        

 sensor_msgs/CompressedImage    image               # the opponent's photo
 uint8                          my_id               # my own ID (e.g., 1) if you are tb401
 uint8                          opp_id              # my opponent ID (e.g., 9) if your enemy is tb409
 float32                        rel_dist            # normed distance of your opponent w.r.t. your frame
 float32                        rel_orient          # orientation (yaw) of your opponent w.r.t. your frame
'''


'''
This is just a general guideline. If you want to write in your own fashion, you can do so as long as it serves the purpose.
'''


class TrackerTwoTB4(Node):

    def __init__(self):

        super().__init__('tracker_two_tb4')

        '''
        The following 3 parameters should be received as the arguments in launch file.
        Those arguments in the launch file should be passed to this node.
        '''

        self.declare_parameter('chaser')   # e.g., tb402       
        self.declare_parameter('enemy')    # e.g., tb407      
        self.declare_parameter('global_frame') # map


        self.chaser = self.get_parameter('chaser').value
        self.enemy= self.get_parameter('enemy').value
        self.global_frame = self.get_parameter('global_frame').value

        enemy_odom_topic = self.enemy + '/odom'
        chaser_odom_topic = self.chaser + '/odom'
        nav_action = self.chaser + '/navigate_to_pose'

        self.H = np.eye(4)  # Use it if you need in constructing your goal_pose

        '''
        The following 3 parameters will be given to you once the final map is constructed.
        As for you now, you can assign some values.
        '''

        self.enemy_offset = ...        # e.g., 0.7 [m]
        self.enemy_fov = ...           # e.g., 40  [deg]. You may need conversion to rad. It really depends on your own program.
        self.enemy_fov_offset = ...    # e.g., 20  [deg]. You may need conversion to rad. It really depends on your own program.

        self.last_goal_x = 0.0   #[m]
        self.last_goal_y = 0.0   #[m]
        self.last_goal_yaw = 0.0 #[rad]
        self.rel_dist_enemy_in_chaser  = 0.0    # [m]
        self.rel_orient_enemy_in_chaser = 0.0   # [deg] (Not to forget conversion from rad to deg). It is easier to check/troubleshoot in deg.

        self.goal_handle = None
        self.pending_goal = False

        self.chaser_base = self.chaser + '/base_link'
        self.enemy_base = self.enemy + '/base_link'
        
        self.tracker_image_sub =  self.create_subscription(
                                                    .........., # Put the compatible message type
                                                    '/'+self.chaser+'/oakd/rgb/preview/image_raw/compressed',
                                                    self.tracker_image_callback,
                                                    10
                                                )

        self.tracker_report_pub = self.create_publisher( ... , # To put your custom msg
                                                        '/'+self.chaser+'/tracker_report',10) # Don't change the topic name

        # Although '/'+self.chaser+'/tracker_report' has image data. It cannot be directly used in RViz. In addition to your custom message, publish it separately as well.
        self.tracker_report_img_pub = self.create_publisher(CompressedImage,
                                                            '/'+self.chaser+'/tracker_report_img/compressed',10) # compressed is necessary as a part of topic (Dont change the topic name)

 
        self.tracker_image = None
       
        # tf 
        self.tf_buffer = Buffer()
        self.tf_listener = ... (self.tf_buffer, self)   # To use TransformListener
        self.tf_broadcaster = ... (self)                # To use TransformBroadcaster
      
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            nav_action
        )

        self.timer = self.create_timer(0.2, self.update_goal) # 20 Hz
        self.get_logger().info("Tracker Two TB4 initialized")

    def tracker_image_callback(self,msg):

        self.tracker_image = ... # To pass the image 


    def homogeneous_T(self,pos,yaw):
        '''
        If you need homogeneous transformation (4x4) matrix. You may refer to the lecture slides. 
        You can approach us if you could not identify it in the lecture slides.
        '''

        H = np.eye(4)
        c_yaw = np.cos(yaw)
        s_yaw = np.sin(yaw)

        # Complete the H matrix.
        H[0:2,0:2]= np.array([[..., ...],
                              [..., ...]])
        H[0:3,3] = ...

        return H


    def sign(self,k):
        '''
        You may use it if you need it.
        '''
        if k < 0:
            return -1
        else:
            return 1


    def update_goal(self):
        '''
        This method is to update the goal if required.
        '''

        try:
            # Use tf lookup transform to get enemy pose in global_frame
            enemy_tf = ...........

        except Exception as e:
            self.get_logger().warn(f"Enemy TF failed: {e}")
            return
        
        try:
            #  Use tf lookup transform to get enemy pose in chaser frame
            rel_tf_enemy_in_chaser =  ..............

        except Exception as e:
            self.get_logger().warn(f"Relative TF failed: {e}")
            return

        try:
            # Use tf lookup transform to get chaser in enemy frame
            rel_tf_chaser_in_enemy = ...............

        except Exception as e:
            self.get_logger().warn(f"Relative TF failed: {e}")
            return

        

        enemy_x = enemy_tf.transform.translation.x
        enemy_y = enemy_tf.transform.translation.y

        # get relative x and y of enemy in chaser frame
        rel_x_enemy_in_chaser = ........
        rel_y_enemy_in_chaser = ........

        # get relative quaternion of enemy in chaser frame
        rel_q_enemy_in_chaser = ..............

        # Convert from quaternion to Euler angle (yaw angle)
        _, _, rel_yaw_enemy_in_chaser = euler_from_quaternion([rel_q_enemy_in_chaser.x,
                                                               rel_q_enemy_in_chaser.y, 
                                                               rel_q_enemy_in_chaser.z, 
                                                               rel_q_enemy_in_chaser.w])

        # Calculate rel_dist and rel_orient which are parts of your custom message.
        self.rel_dist_enemy_in_chaser   = .......                                                    
        self.rel_orient_enemy_in_chaser = .......

        
        

        # get relative x and y of chaser in enemy frame
        rel_x_chaser_in_enemy = ............
        rel_y_chaser_in_enemy = ............
        

        

        # To check if the goal (x,y) is in enemy field of view (FOV)
        if ...........................................................: # write your "if" statement if your  
            self.get_logger().warn("In Enemy FOV....")
            # It will be important for you to know if you are in enemy FOV.
            # If so, you might want to take some actions (: not referring to ROS action)

        # This H_enemy_in_map might be useful if you want to map anything from enemy to map
        H_enemy_in_map = self.homogeneous_T(np.array([enemy_x,enemy_y,0]),yaw)

        '''
            Use maths to decide your goal pose (x_d, y_d, theta_d).
            H_d_in_map is the homogeneous transformation which has all information from desired_frame (goal frame) to map frame
        '''
       
        H_d_in_map = H_enemy_in_map @ ................... # @ is a symbol for multiplication


        '''
         From H_d_in_map, you can extract goal_yaw using the following approach. How about goal_x and goal_y?
        '''
        R_d_in_map = R.from_matrix(H_d_in_map[0:3,0:3]) 
        _, _,  goal_yaw = R_d_in_map.as_euler('xyz', degrees=False)

        # To calculate relative distance between the enemy and goal
        rel_dist_bet_enemy_and_goal = math.sqrt((enemy_x- goal_x)**2 + (enemy_y - goal_y)**2)

        '''
        You might want to check if your enemy is far from you or out of sight.
        If so, you might want to send the latest goal.
        If not, you might want to keep the same goal (last goal).
        '''
        if ...................................................................: # Write your "if" statement
            goal_x   = goal_x
            goal_y   = goal_y
            goal_yaw = goal_yaw
            self.send_goal(goal_x, goal_y, goal_yaw)
        else:
            
            goal_x   = self.last_goal_x   
            goal_y   = self.last_goal_y  
            goal_yaw = self.last_goal_yaw

    

        self.last_goal_x   = goal_x
        self.last_goal_y   = goal_y
        self.last_goal_yaw = goal_yaw


       # Using TF broacaster, publish your goal_pose so that it is easier for you to check if your maths is correct.

   

    def send_goal(self, x, y, yaw):

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not available")
            return

        self.pending_goal = True

        goal_msg = NavigateToPose.Goal()

        # Write your goal_msg
        ......
        ......
        ....

        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f}, {yaw:.2f})")

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.pending_goal = False
            return

        self.get_logger().info("Goal accepted")

        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):

       
        if self.pending_goal is True:
            result = future.result()
            status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")
            if self.tracker_image is None:
                self.get_logger().info("Goal succeeded but No Image Available.")
                return
            
            # When your goal is successful, you need to update your custom message
            ..................
            ................
            ...................
            .................



            self.tracker_report_pub.publish(.................) # after the update, do not forget to publish it.



            # Similarly, you should publish the enemy photo separately. 
           
            self.tracker_report_img_pub.publish(.........)


        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted → sending alternative route")
          
            

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled → sending alternative route")
          

        else:
            self.get_logger().warn(f"Unknown status: {status}")
           


def main():
    rclpy.init()
    node = TrackerTwoTB4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()