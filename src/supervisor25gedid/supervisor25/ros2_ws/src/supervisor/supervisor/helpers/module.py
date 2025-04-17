"""
Module class to launch and shutdown modules (launch files)
"""
from enum import Enum
from typing import Optional
import subprocess
import rclpy
from asurt_msgs.msg import NodeStatus
from .intervalTimer import IntervalTimer
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from rclpy.node import Node




class ModuleState(Enum):
    """
    Enum class for the supervisor's state
     "starting": 0,
        "ready": 1,
        "running": 2,
        "error": 3,
        "shutdown": 4,
        "unresponsive": 5
    }
    """
    Ready = 1
    Running = 2
    Error = 3
    Shutdown =4

class Module(Node):  # Module should inherit from Node properly
    """
    Launches a module (launch file) and checks if it is alive using the optional heartbeat topic
    """

    def __init__(
        self,
        pkg: str,
        launchFile: str,
        heartbeat: Optional[str] = None,
        isHeartbeatNodestatus: bool = True,
        shutdown_callback=None,
        restart_callback=None
        
    ) -> None:
        # Ensure ROS 2 is initialized
        if not rclpy.ok():
            rclpy.init()

        # Correctly initialize the Node
        super().__init__("module")
        self.shutdown_callback= shutdown_callback
        self.restart_callback=restart_callback
        self.pkg = pkg
        self.launchFile = launchFile
        self.state = NodeStatus.SHUTDOWN
        self.moduleHandle = None
        self.scheduleRestart = False
        self.hasHeartbeat = heartbeat is not None
        self.rate = 0.0
        self.launch_process =None
        self.last_heartbeat_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.heartbeat_count = 0.0
        self.new_heartbeat_rate=0.0 
        try:
            # Ensure package and launch file are not None
            assert self.pkg is not None and self.launchFile is not None
        except AssertionError:
            raise ValueError("Supervisor.Module: must have a valid package and launch file.")
        
        # Setup heartbeat if provided
        self.get_logger().info(f"hasHeartbeat: {self.hasHeartbeat}")
        if self.hasHeartbeat:
            try:
                if isHeartbeatNodestatus:
                    self.get_logger().info(f"📡 Attempting to subscribe to {heartbeat} with type NodeStatus...")
                    self.create_subscription(NodeStatus, heartbeat, self.heartbeat_callback, 10)
                    self.get_logger().info(f"Subscribed to heartbeat topic: {heartbeat}")
                else:
                    self.create_subscription(NodeStatus, heartbeat, self.msg_callback, 10)  # Ensure valid msg type
                    
                self.get_logger().info(f"dakhalna lehad hena ")
                
                self.heartbeat_rate_thread = IntervalTimer(1, self.update_heartbeat_rate)
                self.heartbeat_rate_thread.start()  # <-- Start the thread
               # self.get_logger().info("✅ IntervalTimer started for heartbeat monitoring.")
                self.last_heartbeat_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.heartbeat_count = 0.0
            except Exception as e:
                self.get_logger().error(f"Failed to create subscription: {e}")

    def __repr__(self) -> str:
        return f"{self.pkg} {self.launchFile}"



    def shutdownmodule(self) -> None:
        """
        Shuts down the module.
        """
        if self.moduleHandle is not None:
            try:
                self.moduleHandle.shutdown()
                if self.hasHeartbeat:
                    self.heartbeat_rate_thread.stop()
                    self.heartbeat_rate_thread = IntervalTimer(1, self.update_heartbeat_rate)
                self.moduleHandle = None
                self.state = ModuleState.SHUTDOWN
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
                self.state=ModuleState.Error

    def heartbeat_callback(self, msg: NodeStatus) -> None:
        self.get_logger().info(f" da5alnaaaaa heartbeat_callback Heartbeat received for module: {self.pkg}, Status: {msg.status}, Count before: {self.heartbeat_count}")
        self.state = msg.status
        self.heartbeat_count += 1.0
        self.get_logger().info(f"✅ Heartbeat count after update: {self.heartbeat_count}")

# msg_callback & heartbeat_callback?? very alike barely know the diff written in notes
    def msg_callback(self, _: NodeStatus) -> None:
        """
        Callback for the topic the module publishes (used if no heartbeat topic is available)
        """
        self.get_logger().info("Heartbeat received in msg_callback")
        self.state = NodeStatus.RUNNING
        self.heartbeat_count += 1.0

    def update_heartbeat_rate(self) -> None:
        """
        Updates the heartbeat rate, called using a looping thread
        """

        self.get_logger().info("🟢 update_heartbeat_rate function is being called...")  # Debugging log
        #the if is for testing, delete when done
        """ 
        if NodeStatus.UNRESPONSIVE:
            self.restart()
        #just testing ^
        """
        try:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            time_diff = current_time - self.last_heartbeat_time
            self.last_heartbeat_time = current_time
            
            #self.get_logger().info(f"Time diff: {time_diff}")
            

            new_heartbeat_rate = self.heartbeat_count / (time_diff + 0.001)  # Avoid division by zero
            self.heartbeat_count = 0

            beta = 0.3  # Smoothing factor
            self.rate = beta * self.rate + (1.0 - beta) * new_heartbeat_rate
            
            if new_heartbeat_rate < 1:
                #self.get_logger().warn("❌ Node marked as UNRESPONSIVE. Restarting...")
                self.state=ModuleState.Error
                self.get_logger().info(f"will now shutdown")
                if self.shutdown_callback:
                    self.shutdown_callback()  # 👈 call the function provided by MissionLauncher
                
            
        except Exception as e:
            self.get_logger().error(f"Error updating heartbeat rate: {e}")
            

    def launch(self) -> None:
        """
        Launch the module using ROS 2 launch files
        """
        self.get_logger().info(f"Launching module: {self.pkg}/{self.launchFile}")
        

        try:
            pkg_share_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH', ''), self.pkg, 'launch')
            launch_file_path = os.path.join(pkg_share_dir, self.launchFile)

        
        # Start the ROS 2 launch process in a separate process
            self.launch_process = subprocess.Popen(
                ["ros2", "launch", self.pkg, self.launchFile],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.get_logger().info(f"Launched {self.pkg}/{self.launchFile} (PID: {self.launch_process.pid})")
            self.state= ModuleState.Running

            if self.hasHeartbeat:
                # Start a heartbeat thread
                self.heartbeat_thread = self.start_heartbeat()

        except Exception as e:
            self.get_logger().error(f"Failed to launch module: {e}")
            self.state = ModuleState.Error


    def run_launch_service(self):
        """
        Runs the LaunchService in a separate thread but ensures it's handled properly.
        """
        try:
            self.get_logger().info("🔄 Running ROS 2 spin loop for subscriptions...")
            rclpy.spin(self)
        except Exception as e:
            self.get_logger().error(f"Error while running launch service: {e}")


    def __del__(self) -> None:
        """
        Destructor to ensure proper shutdown
        """
        try:
            self.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error in destructor: {e}")

    def shutdownlaunchfile(self):
        """
        Shutdowns the module safely
        """
        if self.launch_process:
            try:
                self.get_logger().info(f"han2fel  launch process ( {self.launch_process})")
                self.launch_process.terminate()
                self.launch_process.wait()
                self.get_logger().info("Launch process shut down successfully")
                self.state = ModuleState.SHUTDOWN
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
                self.state=ModuleState.Error