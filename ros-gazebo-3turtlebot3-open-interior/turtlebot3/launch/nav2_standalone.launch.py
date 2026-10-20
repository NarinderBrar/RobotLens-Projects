import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- Launch description part ---
def generate_launch_description():
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator'
    ]

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,

        # Static transform publisher to align map -> odom frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Kinematic simulator running inline (handles standalone mode)
        ExecuteProcess(
            cmd=[sys.executable, __file__],
            output='screen'
        ),

        # The 5 core Nav2 servers
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # Lifecycle manager to coordinate transitions of the 5 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                params_file,
                {'node_names': lifecycle_nodes},
                {'autostart': True}
            ]
        )
    ])


# --- Inline Kinematic Simulator Node ---
if __name__ == '__main__':
    import math
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist, Quaternion
    from rclpy.node import Node as RosNode
    from tf2_ros import TransformBroadcaster
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry

    TARGET_FRAME_ID = "odom"
    TARGET_CHILD_FRAME = "base_footprint"

    WHEEL_RADIUS = 0.033
    WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']

    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    class InlineKinematicSim(RosNode):
        def __init__(self):
            super().__init__("turtlebot_kinematic_sim")
            self.broadcaster = TransformBroadcaster(self)
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

            self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

            # Kinematic simulator state
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.wheel_left_angle = 0.0
            self.wheel_right_angle = 0.0

            self.publish_rate = 30.0
            self.dt = 1.0 / self.publish_rate
            self.timer = self.create_timer(self.dt, self._tick)

            self.get_logger().info("Inline turtlebot_kinematic_sim started (standalone kinematic mode)")

        def _cmd_vel_callback(self, msg):
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z

        def _tick(self):
            stamp = self.get_clock().now().to_msg()

            # Unicycle model integration
            self.x += self.linear_vel * math.cos(self.theta) * self.dt
            self.y += self.linear_vel * math.sin(self.theta) * self.dt
            self.theta += self.angular_vel * self.dt

            # Estimate wheel spin angles
            wheel_rot = (self.linear_vel / WHEEL_RADIUS) * self.dt
            self.wheel_left_angle += wheel_rot - (self.angular_vel * 0.08 / WHEEL_RADIUS) * self.dt
            self.wheel_right_angle += wheel_rot + (self.angular_vel * 0.08 / WHEEL_RADIUS) * self.dt

            # Publish TF odom -> base_footprint
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = TARGET_FRAME_ID
            t.child_frame_id = TARGET_CHILD_FRAME
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quaternion(self.theta)
            self.broadcaster.sendTransform(t)

            # Publish /odom
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = TARGET_FRAME_ID
            msg.child_frame_id = TARGET_CHILD_FRAME
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation = yaw_to_quaternion(self.theta)
            msg.twist.twist.linear.x = self.linear_vel
            msg.twist.twist.angular.z = self.angular_vel
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.01
            msg.pose.covariance[35] = 0.02
            self.odom_pub.publish(msg)

            # Publish /joint_states for wheel visual spin
            js = JointState()
            js.header.stamp = stamp
            js.name = WHEEL_JOINTS
            js.position = [self.wheel_left_angle, self.wheel_right_angle]
            self.joint_pub.publish(js)

    rclpy.init()
    node = InlineKinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
