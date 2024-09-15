#!/usr/bin/env python3
from math import sqrt, pi
import sys
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Import the Bool message

# Define goal points
goal_points = [
     (-0.28, -0.24), (2.8575, -0.24), (-0.28, 2.885),  (2.8575, 2.885), (2.8575, 6.01),
    (-0.28, 6.01), (-0.28, 9.135), (-0.28, 12.26),  (2.8575, 9.135),  (5.995, 9.135),
    (9.1325, 9.135),  (9.1325, 6.01), (2.8575, 12.26),   (5.995, 12.26),   (9.1325, 12.26),
    (12.27, 12.26), (12.27, 9.135), (12.27, 6.01),  (12.27, 2.885),  (12.27, -0.24),
    (9.1325, 2.885), (5.995, 2.885), (5.995, 6.01),  (9.1325, -0.24),  (5.995, -0.24)
]

success = True

def main():
    global auto_chaos
    global nav_to_pose_client
    
    status = 4

    rclpy.init()

    auto_chaos = rclpy.create_node('waypoint_explorer')

    # Set Initial Pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = auto_chaos.get_clock().now().to_msg()

    # Define the initial pose of the robot (e.g., at the origin)
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    print("Setting initial pose of the robot...")

    # Set initial pose in RViz or simulation
    set_initial_pose(auto_chaos, initial_pose)

    # Create Action Client object with desired message type and action name
    nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')

    # Wait for action server to come up
    while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        print("Server still not available; waiting...")

    # Send the robot to each goal point
    for position_coords in goal_points:
        if rclpy.ok():
            try:
                position = generatePosition(position_coords)
                orientation = generateOrientation()
                goal_handle = sendGoal(position, orientation)
                status = checkResult(goal_handle)
                  # After reaching the goal, rotate 360 degrees
                rotate_360(auto_chaos)
                
            except KeyboardInterrupt:
                print("Shutdown requested... complying...")
                break

    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()

def set_initial_pose(node, initial_pose):
    """
    Publish the initial pose to the /initialpose topic so that the robot knows where it is.
    This ensures proper localization before starting navigation.
    """
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    
    # Wait for the publisher to be ready
    while initial_pose_pub.get_subscription_count() == 0:
        print("Waiting for subscribers to /initialpose...")
        #rclpy.spin_once(node)
    
    # Convert PoseStamped to PoseWithCovarianceStamped
    initial_pose_with_cov = PoseWithCovarianceStamped()
    initial_pose_with_cov.header = initial_pose.header
    initial_pose_with_cov.pose.pose = initial_pose.pose

    initial_pose_with_cov.pose.covariance = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
]



    # Publish the initial pose
    #initial_pose_pub.publish(initial_pose)
    
    
    # Publish the initial pose with covariance
    initial_pose_pub.publish(initial_pose_with_cov)
    print("Initial pose set!")


def sendGoal(position, orientation):
    """Create action goal object and send to action server, check if goal accepted"""
    global auto_chaos
    global nav_to_pose_client

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
    goal.pose.pose.position = position
    goal.pose.pose.orientation = orientation

    print(f"Sending new goal => X: {goal.pose.pose.position.x} Y: {goal.pose.pose.position.y}")

    send_goal_future = nav_to_pose_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        print("Goal was rejected")
        nav_to_pose_client.destroy()
        auto_chaos.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    print("Goal Accepted!")
    return goal_handle


def checkResult(goal_handle):
    """Check for task completion while blocking further execution"""
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(auto_chaos, get_result_future)

    status = get_result_future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        print("Reached Goal!!!")
    else:
        print(f"Goal failed with status: {status}")

    return status


def generatePosition(coords):
    """Use predefined coordinates for position"""
    position = Point()
    position.x = coords[0]
    position.y = coords[1]
    position.z = 0.0
    return position


def generateOrientation():
    """Set a default orientation (facing straight forward)"""
    quat = Quaternion()
    quat.w = 1.0  # No rotation in yaw
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat

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
    duration = 3 * pi / angular_speed  # time = angle / angular_speed
    
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



if __name__ == '__main__':
    main()
