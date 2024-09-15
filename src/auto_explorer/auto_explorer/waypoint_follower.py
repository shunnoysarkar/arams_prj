#!/usr/bin/env python3
from math import sqrt, pi
import sys
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import Bool  # Import the Bool message
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Define goal points
goal_points = [
    (-0.28, -0.24), (2.8575, -0.24), (-0.28, 2.885), (2.8575, 2.885), (2.8575, 6.01),
    (-0.28, 6.01), (-0.28, 9.135), (-0.28, 12.26), (2.8575, 9.135), (5.995, 9.135),
    (9.1325, 9.135), (9.1325, 6.01), (2.8575, 12.26), (5.995, 12.26), (9.1325, 12.26),
    (12.27, 12.26), (12.27, 9.135), (12.27, 6.01), (12.27, 2.885), (12.27, -0.24),
    (9.1325, 2.885), (5.995, 2.885), (5.995, 6.01), (9.1325, -0.24), (5.995, -0.24)
]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Flag to stop navigation when 3 images are detected
        self.stop_navigation = False

        # Subscribe to /image_detection_complete
        self.create_subscription(Bool, '/image_detection_complete', self.image_detection_callback, 10)

        # Set up action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def image_detection_callback(self, msg):
        """Callback when notified by yolo_to_ros node that 3 images were detected"""
        if msg.data:
            print("Received notification to return to (0, 0).")
            self.stop_navigation = True
            self.navigate_back_to_origin()
        else:
            print("Received notification, but the data is False")

    def navigate_back_to_origin(self):
        """Navigate the robot back to the starting point (0, 0)"""
        goal_position = Point(x=0.0, y=0.0, z=0.0)
        goal_orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        
        
        print(f"Sending goal to return to (0, 0): Position -> X: {goal_position.x}, Y: {goal_position.y}, Orientation -> W: {goal_orientation.w}, Z: {goal_orientation.z}")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            print("Navigation action server is not available.")
            return
        goal_handle = self.send_goal(goal_position, goal_orientation)
        # Ensure the goal is sent properly
        if goal_handle:
            self.check_result(goal_handle)
        else:
            print("Failed to send the goal to (0,0).")
    
    def send_goal(self, position, orientation):
        """Create and send a navigation goal"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation = orientation

        print(f"Sending new goal => X: {goal.pose.pose.position.x}, Y: {goal.pose.pose.position.y}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("Goal was rejected by the action server.")
            return None
        
        print("Goal Accepted!")
        return goal_handle

    def check_result(self, goal_handle):
        """Check the result of the sent goal"""
        get_result_future = goal_handle.get_result_async()
        
        # Adding a timeout to prevent the robot from freezing
        timeout_sec = 30.0  # Timeout for goal completion (adjust as needed)
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=timeout_sec)
        status = get_result_future.result().status if get_result_future.result() else None

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Reached Goal!!!")
        elif status is None:
            print(f"Goal execution timed out after {timeout_sec} seconds.")
        else:
            print(f"Goal failed with status: {status}")
    
    def rotate_360(node):
    
    
        """
        Function to rotate the robot 360 degrees by publishing velocity commands to /cmd_vel.
        """
        # Publisher for /cmd_vel to control the robot's velocity
        vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
        # Create a new Twist message
        twist = Twist()
    
        # Set the angular velocity for rotating in the z-axis (yaw rotation)
        angular_speed = pi / 4  # Adjust as needed (radians per second)

        # Calculate the total duration needed to complete 360 degrees
        duration = 2.5 * pi / angular_speed  # time = angle / angular_speed
    
        twist.angular.z = angular_speed  # Rotate in the z-axis
    
        # Start rotation
        print("Starting 360-degree rotation...")
    
        start_time = node.get_clock().now().to_msg().sec
        while (node.get_clock().now().to_msg().sec - start_time) < duration:
            vel_pub.publish(twist)
            sleep(0.3)
    
        # Stop rotation after completing the full circle
        twist.angular.z = 0.0
        vel_pub.publish(twist)
    
        print("Rotation complete!")
        

def main():
    rclpy.init()
    explorer = WaypointFollower()

    # Continue navigation to waypoints unless stopped
    for coords in goal_points:
        if explorer.stop_navigation:
            print("explorer.stop_navigation is now True. breaking for loop...")
            break

        position = Point(x=coords[0], y=coords[1], z=0.0)
        orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        goal_handle = explorer.send_goal(position, orientation)
         # Ensure the goal is sent before checking result
        if goal_handle:
            explorer.check_result(goal_handle)
        else:
            print(f"Failed to send goal to X: {coords[0]}, Y: {coords[1]}")

        # After reaching the goal, rotate 360 degrees
        explorer.rotate_360()
    
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
