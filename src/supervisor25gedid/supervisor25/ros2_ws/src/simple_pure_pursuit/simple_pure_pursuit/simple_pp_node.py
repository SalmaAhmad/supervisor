#!/usr/bin/env python3
"""
Initialization Pure Pursuit node for vehicle control
"""
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
import threading
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, String
from .simple_pure_pursuitController import SimplePurePursuit
from tf_helper.TFHelper import TFHelper
from asurt_msgs.msg import NodeStatus
from tf_helper.StatusPublisher import StatusPublisher

class SimplePurePursuitNode(Node):  # type: ignore[misc]
    """
    Class to run the simple pure pursuit controller node
    Main function for simple pure pursuit vehicle control node, subscribes to
    waypoints and publishes control actions
    """

    def __init__(self) -> None:
        """
        Initialization of the simple pure pursuit node
        """
        super().__init__("simple_pure_pursuit_node")
        self.controller = SimplePurePursuit(self)

        self.targetSpeed = 5.0
        controlTopic = "/ackr"
        pathTopic = "/acc_planning"

        self.rate = self.create_rate(1)  # self.controlRate)
        self.tfHelper = TFHelper(self)
        self.markerPub = self.create_publisher(Marker, "/marker_viz", 10)
        self.controlActionPub = self.create_publisher(AckermannDriveStamped, controlTopic, 10)
        self.steeringPub = self.create_publisher(Float32, "/steer", 10)
        self.pathPub = self.create_publisher(Path, "/debug", 10)
        self.statusPub = StatusPublisher("status/simple_pure_pursuit", self)
        self.statusPub.starting()
        self.statusPub.ready()
        # self.create_subscription(Path, pathTopic, self.pathCallback, 10)
        # self.create_timer(0.01, self.testing)
        # self.create_timer(0.01, self.controller.run)


    def testing(self):
        self.statusPub.running()
        self.get_logger().info("da5al")
    def pathCallback(self, path: Path) -> None:
        """
        Callback function for path msg
        """
        path = self.tfHelper.transformPathMsg(path, "Fr1A")
        self.pathPub.publish(path)
        self.get_logger().info("Path callback triggered")
        if len(path.poses) > 0:
            self.controller.add(path)

            delta, ind = self.controller.purepursuitSteercontrol()
            controlMsg = AckermannDriveStamped()
            controlMsg.drive.speed = self.targetSpeed
            controlMsg.drive.steering_angle = delta
            controlMsg.header.stamp = self.get_clock().now().to_msg()
            self.visualizer(ind)
            self.controlActionPub.publish(controlMsg)
            self.get_logger().info("Publishing control action")

    def visualizer(self, ind: int) -> None:
        """
        Visualize the targeted waypoint along the path

        Parameters:
        ind (int): Target index
        """
        if len(self.controller.xList) > 0:
            vizPoint = Marker()
            vizPoint.header.stamp = self.get_clock().now().to_msg()
            vizPoint.header.frame_id = "Fr1A"
            vizPoint.ns = "pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = self.controller.xList[ind]
            vizPoint.pose.position.y = self.controller.yList[ind]
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
            self.markerPub.publish(vizPoint)

    # def publish_status(self):
    #     """
    #     Publish status message at 10 Hz
    #     """
    #     rate = self.create_rate(10)
    #     while rclpy.ok():
    #         status_msg = String()
    #         status_msg.data = "Simple Pure Pursuit Node Running"
    #         self.statusPub.publish(status_msg)
    #         self.get_logger().info("Status message published")
    #         rate.sleep()


def main(args=None) -> None:  # type: ignore[no-untyped-def]
    """
    Main function for simple pure pursuit vehicle control node
    """
    rclpy.init(args=args)

    simplePPNode = SimplePurePursuitNode()

    # control_thread = threading.Thread(target=rclpy.spin, args=(simplePPNode,), daemon=True)
    # status_thread = threading.Thread(target=simplePPNode.publish_status, daemon=True)

    # control_thread.start()
    # status_thread.start()

    # control_thread.join()
    # status_thread.join()
    # rate = simplePPNode.create_rate(10)
    # status = StatusPublisher("/status/simple_pure_pursuit", simplePPNode)

    # status.starting()
    # status.ready()
    try:
        rclpy.spin(simplePPNode)
    finally:
        rclpy.shutdown()
    # while rclpy.ok:
        
    #     rate.sleep()
        
    #     # Publish heartbeat to show the module is running
    #     status.running()

#  rclpy.init(args=args)

#     simplePPNode = SimplePurePursuitNode()

#     status = StatusPublisher("status/simple_pure_pursuit", simplePPNode,10)

#     control_thread = threading.Thread(target=rclpy.spin, args=(simplePPNode,), daemon=True)
   
#     control_thread.start()
#     # control_thread.join()

#     rate = simplePPNode.create_rate(10)

#     status.starting()

#     # Publish heartbeat to show the module is ready
#     status.ready()

#     while rclpy.ok:
        
#         rate.sleep()
#         out = simplePPNode.run()

#         if out is None:
#             continue
    
#     status.running()

if __name__ == "__main__":
    main()
