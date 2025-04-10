import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
import math
from tf_transformations import euler_from_quaternion

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_control_signals)

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
        self.target_speed = 5
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0
        self.dt = 0.1
        self.error_sum = 0.0

        #search target index function initialization
        self.firstFlag = True
        self.target_index = 0
        

        #simplePurepursuit function initialization
        self.lookahead_distance = 2.5


        
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
        throttle_msg = Float32()
        throttle_msg.data = self.pid_controller()
        self.throttle_pub.publish(throttle_msg)


    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True
        self.first_element = [t[0] for t in self.waypoints]
        

    def pid_controller(self):
        error = self.target_speed - self.velocity
        p_term = self.kp * error
        self.error_sum += error
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.prev_error) / self.dt
        control_signal = p_term + i_term + d_term
        self.prev_error = error
        control_signal= max(0.0, min(1.0, control_signal))
        return control_signal


    def search_target_point(self):
        min_distance = float('inf')
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
                if self.x > self.first_element[self.target_index]:
                    continue
                else:
                    break
        return self.target_index
    
    @staticmethod
    def calculate_distance(point1, point2):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return math.sqrt(delta_x ** 2 + delta_y ** 2)


    def simplePurepursuit(self):
        target_index = self.search_target_point()
        target_waypoint = self.waypoints[target_index]
        tx, ty = target_waypoint
        dx = tx - self.x
        dy = ty - self.y
        if target_index == len(self.waypoints) - 1:
                return 0
        alpha = math.atan2(dy, dx) - self.yaw
        lookahead_angle = math.atan2(2 * 0.5 * math.sin(alpha) / self.lookahead_distance, 1)
        steering_angle = max(-0.5, min(0.5, lookahead_angle))
        steering_angle = math.degrees(lookahead_angle)
        return steering_angle


    def publish_control_signals(self):
        if self.pathFlag == True :
            steering_angle = Float32()
            steering_angle.data = self.simplePurepursuit()
            self.steer_pub.publish(steering_angle)
            if len(self.waypoints) > 0 and self.search_target_point == len(self.waypoints) - 1:
                throttle = 0
            else:
                throttle = Float32()
                throttle.data = self.pid_controller()
            self.throttle_pub.publish(throttle)
            log = "tracking waypoint: " + str(self.target_index) + str(self.waypoints[self.target_index]) + str(self.velocity)
            self.get_logger().info(log)

            '''
            #plot velocity response
            self.velocity = np.reshape(self.velocity, (100,1))
            plt.plot(self.t_sim,self.velocity)
            plt.xlabel('time')
            plt.ylabel('velocity')
            plt.title('velocity response')
            plt.grid(True)
            plt.show()
            #ValueError: x and y must have same first dimension, but have shapes (100,) and (1,)
            #plt.plot(self.t_sim,self.velocity)
            '''

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()