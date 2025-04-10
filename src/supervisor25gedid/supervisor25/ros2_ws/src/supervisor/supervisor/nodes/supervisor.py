"""
Supervisor main module
"""
from typing import Optional
from enum import Enum
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from eufs_msgs.msg import CanState
from geometry_msgs.msg import TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from supervisor.helpers.missionLauncher import MissionLauncher  # type: ignore
from supervisor.helpers.visualizer import Visualizer  # type: ignore
from asurt_msgs.msg import NodeStatus  # Ensure correct message import




class SuperState(Enum):
    """
    Enum class for the supervisor's state
    """
    WAITING = 0
    LAUNCHING = 1
    READY = 2
    RUNNING = 3
    STOPPING = 4
    FINISHED = 5


class Supervisor(Node):  # pylint: disable=too-many-instance-attributes
    """
    This class is the supervisor's main class
    """

    def __init__(
        self,
        rosCanCmdTopic: str,
        drivingFlagTopic: str,
        missionFlagTopic: str,
        markersTopic: Optional[str] = None,
        btnTopic: Optional[str] = None
    ) -> None:
        super().__init__("Supervisor")
        self.asState = CanState.AS_OFF
        self.amiState = CanState.AMI_NOT_SELECTED
        self.isFinished = False
        self.superState = SuperState.WAITING
        self.maxStopVelTh = 0.1
        self.currentVel = 0
        self.vel = 0
        self.steer = 0
        
        self.drivingFlagPub = self.create_publisher(Bool, drivingFlagTopic, 10)
        self.missionFlagPub = self.create_publisher(Bool, missionFlagTopic, 10)
        self.cmd = self.create_publisher(AckermannDriveStamped, rosCanCmdTopic, 10)
        self.heartbeatSub = self.create_subscription(NodeStatus, "/module_heartbeat", self.heartbeatCallback, 10)
        # if markersTopic is not None and btnTopic is not None:
        #     self.visualizer = Visualizer(markersTopic, btnTopic)
        #     self.launcher = MissionLauncher(self.visualizer)
        # else:
        self.get_logger().info("No visualizer")
        self.launcher = MissionLauncher()
    
    def heartbeatCallback(self, msg: NodeStatus) -> None:
        """
        Callback to receive heartbeat messages from modules.
        """
        self.get_logger().info(f"Received Heartbeat from {msg.node_name}, Status: {msg.status}")
    def run(self) -> None:
        """
        Do the state machine transitions and actions
        """

        # Update launcher
        if self.superState != SuperState.WAITING:
            self.launcher.update()

        # Do transitions
        if self.superState == SuperState.WAITING:
            if self.amiState != CanState.AMI_NOT_SELECTED:
                self.superState = SuperState.LAUNCHING
                self.launcher.launch(self.amiState)
                
        elif self.superState == SuperState.LAUNCHING:
            self.get_logger().info(f"Launcher ready a33333: {self.launcher.isReady()}")
            if self.launcher.isReady():
                self.superState = SuperState.READY
        elif self.superState == SuperState.READY:
            if self.asState == CanState.AS_DRIVING:
                self.superState = SuperState.RUNNING
        elif self.superState == SuperState.RUNNING:
            if self.isFinished:
                self.superState = SuperState.STOPPING
        elif self.superState == SuperState.STOPPING:
            if self.currentVel < self.maxStopVelTh:
                self.superState = SuperState.FINISHED

        #Publish
        self.publishRosCanMessages()

    def publishRosCanMessages(self) -> None:
        """
        Publishes the command to the car
        """

        if self.superState in (SuperState.WAITING, SuperState.LAUNCHING, SuperState.READY):
            self.missionFlagPub.publish(Bool(data = False))
            self.drivingFlagPub.publish(Bool(data = False))
        elif self.superState in (SuperState.RUNNING, SuperState.STOPPING):
            self.missionFlagPub.publish(Bool(data = False))
            self.drivingFlagPub.publish(Bool(data = True))
        elif self.superState == SuperState.FINISHED:
            self.missionFlagPub.publish(Bool(data = True))
            self.drivingFlagPub.publish(Bool(data = False))

        self.cmd.publish(self.getCmdMessage())

    def getCmdMessage(self) -> AckermannDriveStamped:
        """
        Publishes the command to the car
        after changing the steer and vel
        msg to AckermannDriveStamped
        """
        cmdMsg = AckermannDriveStamped()
        if self.superState in (
            SuperState.WAITING,
            SuperState.LAUNCHING,
            SuperState.READY,
            SuperState.FINISHED,
        ):
            cmdMsg.drive.speed = 0.0
            cmdMsg.drive.steering_angle = 0.0
        elif self.superState == SuperState.RUNNING:
            cmdMsg.drive.speed = self.vel
            cmdMsg.drive.steering_angle = self.steer
        elif self.superState == SuperState.STOPPING:
            cmdMsg.drive.steering_angle = self.steer
            if self.currentVel > 0.1:
                targetVel = 0.5 * self.currentVel
            else:
                targetVel = 0
            cmdMsg.drive.speed = targetVel
        cmdMsg.header.stamp = rclpy.time.Time().to_msg()
        return cmdMsg

    def canStateCallback(self, msg: CanState) -> None:
        """
        Callback to retreive the VCU's state and selected mission
        """
        self.get_logger().info(f"Received CAN state message: {msg.as_state}")
        self.get_logger().info(f"Received CAN state message: {msg.ami_state}")
        self.asState = msg.as_state
        self.amiState = msg.ami_state

    def isFinishedCallback(self, msg: Bool) -> None:
        """
        Callback for the mission's finished flag
        """
        self.get_logger().info(f"Received isFinished message: {msg.data}")
        self.isFinished = msg.data

    # def velCallback(self, msg: Float32) -> None:
    #     """
    #     Callback function for the velocity
    #     """
    #     self.get_logger().info(f"Received velocity message: {msg.data}")
    #     self.vel = msg.data

    # def steerCallback(self, msg: Float32) -> None:
    #     """
    #     Callback function for the steering angle
    #     """
    #     self.get_logger().info(f"Received steering message: {msg.data}")
    #     self.steer = msg.data

    def controlCallback(self, msg: AckermannDriveStamped) -> None:
        self.vel = msg.drive.speed  
        self.steer = msg.drive.steering_angle

    def currentVelCallback(self, msg: TwistWithCovarianceStamped) -> None:
        """
        Callback for the current velocity
        """
        self.get_logger().info(f"Received current velocity message: {msg.twist.twist.linear.x}")
        self.currentVel = msg.twist.twist.linear.x