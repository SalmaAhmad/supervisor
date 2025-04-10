import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        # self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ackr', 10)
        self.state_sub = self.create_subscription(Odometry, '/carmaker/Odometry', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, '/csvtopath', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_drive)
        self.point_marker = self.create_publisher(Marker, "/target_point" ,10)



        #state_callback function initialization
        self.velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = (self.x, self.y, self.yaw)

        #path_callback function initialization
        self.waypoints = []
        self.pathFlag = False

        #pid_controller function initialization
        self.target_speed = 0.0
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0
        self.dt = 0.1
        self.error_sum = 0.0

        #search target index function initialization
        self.firstFlag = True
        self.target_index = 0

        #adaptivePurepursuit function initialization
        self.lookahead_distance = 0.0
        self.lookaheadconstant = 1.1
        self.gain = 0.3

        #speedControl function initialization
        self.minspeed = 3
        self.maxspeed = 6
        self.speed_constant = 29.0
        self.phai = 0.0
        self.steering_angle = 0.0



    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        self.velocity = math.sqrt(Vx ** 2 + Vy ** 2)
        self.x = state.pose.pose.position.x
        self.y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, self.yaw = euler_from_quaternion(orientation_list)
        self.state = (self.x, self.y, self.yaw)


    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True
        self.first_element = [t[0] for t in self.waypoints]
        self.second_element = [t[1] for t in self.waypoints]
    

    def pid_controller(self,steering):
        self.target_speed = self.speedControl(steering)
        error = self.target_speed - self.velocity
        p_term = self.kp * error
        self.error_sum += error
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.prev_error) / self.dt
        self.control_signal = p_term + i_term + d_term
        self.prev_error = error
        self.control_signal= max(-1.0, min(1.0, self.control_signal))
        return self.control_signal


    def search_target_point(self):
        min_distance = float ('inf')
        if self.firstFlag:
            for i , waypoint in enumerate(self.waypoints):
                distance = self.calculate_distance(self.state[:2], waypoint)
                if distance < min_distance:
                    min_distance = distance
                    self.target_index = i
                    self.firstFlag = False
        
        for i in range(self.target_index,len(self.waypoints) - 1):
            distance = self.calculate_distance(self.state[:2], self.waypoints[i])
            if distance > self.lookahead_distance :
                
                self.target_index = i
                break
                
                # self.phai = math.atan2((self.y-self.second_element[self.target_index])/(self.x-self.first_element[self.target_index]),1)
                # if self.x > self.first_element[self.target_index] or abs(self.phai - self.yaw) > 1.55:
                # if self.x > self.first_element[self.target_index]:
                #     continue
                # else:
                #     break
        return self.target_index    

        
    @staticmethod
    def calculate_distance(point1: list, point2: list):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return math.sqrt(delta_x ** 2 + delta_y ** 2)


    def adaptivePurepursuit(self):
        self.lookahead_distance = self.velocity * self.gain + self.lookaheadconstant
        target_index = self.search_target_point()
        target_waypoint = self.waypoints[target_index]
        tx, ty = target_waypoint
        dx = tx - self.x
        dy = ty - self.y
        # if target_index == len(self.waypoints) - 1:
        #         return 0
        alpha = math.atan2(dy, dx) - self.yaw# if target_index == len(self.waypoints) - 1:
        #         return 0
        lookahead_angle = math.atan2(2 * 0.5 * math.sin(alpha) / self.lookahead_distance, 1)
        self.steering_angle = max(-0.5, min(0.5, lookahead_angle))
        return self.steering_angle


    def speedControl(self, steering_angle: float) -> float:
        self.target_speed = self.speed_constant / (abs(steering_angle)*(180/math.pi) + 0.1) # change steering angle to rad
        #targetSpeed: float = map(abs(steering_angle),0,30,3,0.5)
        self.target_speed = min(self.target_speed,self.maxspeed)
        self.target_speed = max(self.target_speed,self.minspeed)
        return self.target_speed
    

    # def publish_control_signals(self):
    #     if self.pathFlag == True :
    #         steering_angle = Float32()
    #         steering_angle.data = self.adaptivePurepursuit()*(180/math.pi)
    #         self.steer_pub.publish(steering_angle)
    #         if len(self.waypoints) > 0 and self.search_target_point() == len(self.waypoints) - 1:  # Added parentheses to the function call
    #             throttle = 0
    #         else:
    #             throttle = Float32()
    #             throttle.data = self.pid_controller(steering_angle.data)
    #         self.throttle_pub.publish(throttle)
    #         log = "tracking waypoint: " + str(self.target_index) + str(self.waypoints[self.target_index]) + " Car Velocity: " + str(self.velocity) + " Lookahead Distance: " + str(self.lookahead_distance) + "Target Speed: " + str(self.target_speed) 
    #         self.get_logger().info(log)

    def publish_drive(self):
        if self.pathFlag == True:
            drive_msg = AckermannDriveStamped()
            
            
            #steering_angle = float()
            steering_angle = self.adaptivePurepursuit() #*(180/math.pi)
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = 1.0
            if len(self.waypoints) > 0 and self.search_target_point() == len(self.waypoints) - 1:  # Added parentheses to the function call
                throttle = 0
                drive_msg.drive.speed = throttle
            else:
                #throttle = float()
                throttle = self.pid_controller(steering_angle)
                drive_msg.drive.speed = self.velocity + throttle
            
            vizPoint = Marker()
            vizPoint.header.frame_id = "map"
            # vizPoint.header.stamp = self.get_clock().now()
            vizPoint.ns = "adaptive_pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = self.first_element[self.target_index]
            vizPoint.pose.position.y = self.second_element[self.target_index]
            vizPoint.pose.position.z = 0.0
            vizPoint.pose.orientation.x = 0.0
            vizPoint.pose.orientation.y = 0.0
            vizPoint.pose.orientation.z = 0.0
            vizPoint.pose.orientation.w = 1.0
            vizPoint.scale.x = 0.5
            vizPoint.scale.y = 0.5
            vizPoint.scale.z = 0.5
            vizPoint.color.r = 1.0
            vizPoint.color.a = 1.0
            



            self.drive_pub.publish(drive_msg)
            self.point_marker.publish(vizPoint)
            log = "tracking waypoint: " + str(self.target_index) + str(self.waypoints[self.target_index]) + " Car Velocity: " + str(self.velocity) + " Lookahead Distance: " + str(self.lookahead_distance) + "Target Speed: " + str(self.target_speed) + "control signal" + str(self.control_signal) + "steering angle" + str(self.steering_angle)
            self.get_logger().info(log)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
