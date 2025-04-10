import os
import json
from typing import List, Dict, Optional
from .intervalTimer import IntervalTimer
import asyncio
import rclpy
from rclpy.node import Node
import time
from eufs_msgs.msg import CanState
from asurt_msgs.msg import NodeStatus
from ament_index_python.packages import get_package_share_directory
from .module import Module
from .visualizer import Visualizer
import subprocess
import signal
import threading
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from functools import partial
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from ..nodes.subb import subb

AMIToConfig = {
    CanState.AMI_DDT_INSPECTION_A: "staticA",
    CanState.AMI_DDT_INSPECTION_B: "staticB",
    CanState.AMI_AUTONOMOUS_DEMO: "autonomousDemo",
    CanState.AMI_AUTOCROSS: "autocross",
    CanState.AMI_SKIDPAD: "skidpad",
    CanState.AMI_ACCELERATION: "acceleration",
    CanState.AMI_TRACK_DRIVE: "trackDrive",
}


def map_state_to_int(state_str: str) -> int:
    state_map = {
        "starting": 0,
        "ready": 1,
        "running": 2,
        "error": 3,
        "shutdown": 4,
        "unresponsive": 5
    }
    return state_map.get(state_str.lower(), -1)  # Return -1 if the state string is not found


class MissionLauncher(Node):
    def __init__(self) -> None:
        super().__init__('mission_launcher_node')
        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules: List[Module] = []
        self.terminal_processes = []  # Change to a list to store process IDs
        self.lastHeartbeatTime = time.time()
        self.heartbeatCount = 0
        self.drivinfgFlagPub = self.create_publisher(Bool, '/supervisor/driving_flag', 10)
        self.rate = 0
        self.state = NodeStatus.SHUTDOWN
        
        """ self.state_simple_pure_pursuit = NodeStatus.SHUTDOWN
        self.heartbeatCountsimple_pure_pursuit = 0

        self.state_accerleration = NodeStatus.SHUTDOWN
        self.heartbeatCountaccerleration = 0
        """
     

    def openConfig(self, fileName: str) -> Dict[str, List[Dict[str, str]]]:
        '''
        Opens a JSON file and returns the content as a dictionary

        Args:
            fileName (str): The name of the JSON file to open
        '''
        with open(
            os.path.join(get_package_share_directory('supervisor'), 'json', fileName),
            encoding="utf-8",
        ) as configFile:
            config = json.load(configFile)
        return config

    def launch(self, mission: int) -> None:
        """
        Launches a mission based on the mission type received
        """
        if self.isLaunched:
            self.get_logger().info("Trying to launch a mission while another mission is running, ignoring")
            return

        if mission == -1:
            config = self.openConfig("testConfig.json")
            self.missionType = "test mission"
        else:
            try:
                config = self.openConfig(AMIToConfig[mission] + ".json")
                self.missionType = AMIToConfig[mission]
            except KeyError as exc:
                raise KeyError(f"Invalid mission type: {mission}") from exc

        for i in config["modules"]:
            self.modules.append(
                Module(i["pkg"], i["launch_file"], i["heartbeats_topic"], bool(i["is_node_msg"]))
            )

        for idx, module in enumerate(self.modules):
            module.launch()
       

    def shutdown(self) -> None:
        for module in self.modules:
            module.shutdown()

        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules = []
         
    from asurt_msgs.msg import NodeStatus



    def isReady(self) -> bool:
        '''
        Checks if all modules are ready
        '''
        self.get_logger().info(f"DEBUG: isReady() called. Modules count: {len(self.modules)}")  # Print module count
        if not self.modules:
            self.get_logger().info("ERROR: self.modules is EMPTY! No modules have been added.")
            return False  # If no modules exist, return False
        
        for module in self.modules:
            # Map the module state to its corresponding integer
            mapped_state = map_state_to_int(module.state)

            # Log debug information
            self.get_logger().info(f"Module {module.pkg} state: {module.state} (mapped integer: {mapped_state})")

            # Compare against the integer representation of RUNNING (2)
            if module.hasHeartbeat and mapped_state != NodeStatus.RUNNING:
                self.get_logger().info(f"Module {module.pkg} is not ready (current state: {module.state})")
                return False  # Not ready

        return True  # All modules are ready


    def update(self) -> None:
        '''
        Updates the mission launcher
        '''
        if not self.isLaunched or not self.visualizer:
            return

        extraText = [
            ["Mission Type", self.missionType, 2],
        ]

        states = ["starting", "ready", "RUNNING", "error", "shutdown", "unreponsive"]

    #main
def main(args=None):
    rclpy.init(args=args)
    node = MissionLauncher()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()