import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from adaptivePurepursuit import AdaptivePurePursuit
from ackermann_msgs.msg import AckermannDrive

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.steer_pub = self.create_publisher(Float32, "steer", 10)
        self.throttle_pub = self.create_publisher(Float32, "throttle", 10)
        self.drive_pub = self.create_publisher(AckermannDrive, "/ackermann_drive", 10)
        self.state_sub = self.create_subscription(Odometry, "/state", self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, "/path", self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_drive)

        self.adaptive_purepursuit = AdaptivePurePursuit(self)

    def state_callback(self, state: Odometry):
        vx = state.twist.twist.linear.x
        vy = state.twist.twist.linear.y
        self.adaptive_purepursuit.velocity = math.sqrt(vx**2 + vy**2)
        self.adaptive_purepursuit.x = state.pose.pose.position.x
        self.adaptive_purepursuit.y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w,
        ]
        _, _, self.adaptive_purepursuit.yaw = euler_from_quaternion(orientation_list)

    def path_callback(self, path: Path):
        self.adaptive_purepursuit.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]
        self.adaptive_purepursuit.path_flag = True
        self.adaptive_purepursuit.first_element = [t[0] for t in self.adaptive_purepursuit.waypoints]

    # def publish_control_signals(self):
    #     steering_angle = Float32()
    #     steering_angle.data = self.purepursuit.adaptivePurepursuit()
    #     throttle = Float32()
    #     throttle.data = self.purepursuit.pid_controller(steering_angle.data)
    #     self.throttle_pub.publish(throttle)
    #     # self.controlActionsPub.publish(throttle)
    #     self.steer_pub.publish(steering_angle)
    #     # self.controlActionsPub.publish(steering_angle)

    def publish_drive(self):
        if self.adaptive_purepursuit.path_flag:
            drive_msg = AckermannDrive()
            # steering_angle = float()
            steering_angle = self.adaptive_purepursuit() * (180 / math.pi)
            drive_msg.steering_angle = steering_angle
            if (
                len(self.adaptive_purepursuit.waypoints) > 0 and self.adaptive_purepursuit.search_target_point() == len(self.adaptive_purepursuit.waypoints) - 1):  # Added parentheses to the function call
                throttle = 0
                drive_msg.speed = throttle
            else:
                # throttle = float()
                throttle = self.adaptive_purepursuit.pid_controller(steering_angle)
                drive_msg.speed = throttle
            self.drive_pub.publish(drive_msg)
            log = (
                "tracking waypoint: "
                + str(self.adaptive_purepursuit.target_index)
                + str(self.adaptive_purepursuit.waypoints[self.adaptive_purepursuit.target_index])
                + " Car Velocity: "
                + str(self.adaptive_purepursuit.velocity)
                + " Lookahead Distance: "
                + str(self.adaptive_purepursuit.lookahead_distance)
                + "Target Speed: "
                + str(self.adaptive_purepursuit.target_speed)
            )
            self.get_logger().info(log)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
