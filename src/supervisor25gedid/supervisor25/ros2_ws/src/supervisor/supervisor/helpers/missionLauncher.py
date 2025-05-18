import os
import json
from typing import List, Dict, Optional
from .intervalTimer import IntervalTimer
from supervisor.helpers.module import ModuleState
import rclpy
from rclpy.node import Node
import time
from eufs_msgs.msg import CanState
from asurt_msgs.msg import NodeStatus
from ament_index_python.packages import get_package_share_directory
from .module import Module
from .visualizer import Visualizer
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from functools import partial
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from ..nodes.subb import subb
from asurt_msgs.msg import NodeStatus

AMIToConfig = {
    CanState.AMI_DDT_INSPECTION_A: "staticA",
    CanState.AMI_DDT_INSPECTION_B: "staticB",
    CanState.AMI_AUTONOMOUS_DEMO: "autonomousDemo",
    CanState.AMI_AUTOCROSS: "autocross",
    CanState.AMI_SKIDPAD: "skidpad",
    CanState.AMI_ACCELERATION: "acceleration",
    CanState.AMI_TRACK_DRIVE: "trackDrive",
}

"""
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
"""

class MissionLauncher(Node):
    # Class variables for singleton pattern
    _instance = None
    _initialized = False
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(MissionLauncher, cls).__new__(cls)
        return cls._instance
    
    def __init__(self) -> None:
        # Only initialize once using the singleton pattern
        if not MissionLauncher._initialized:
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
            MissionLauncher._initialized = True
            
            """ self.state_simple_pure_pursuit = NodeStatus.SHUTDOWN
            self.heartbeatCountsimple_pure_pursuit = 0

            self.state_accerleration = NodeStatus.SHUTDOWN
            self.heartbeatCountaccerleration = 0
            """
            self.get_logger().info("MissionLauncher singleton instance initialized")

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
                Module( i["pkg"], i["launch_file"], i["heartbeats_topic"], bool(i["is_node_msg"]),shutdown_callback=self.handle_module_shutdown ,restart_callback=self.handle_module_restart )
            )

        for idx, module in enumerate(self.modules):
            module.launch()
       
    
    def shutdown(self) -> None:
        for module in self.modules:
            module.shutdownmodule()
            module.shutdownlaunchfile()

        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules = []
         
    def handle_module_shutdown(self):
        self.get_logger().info(" Shutdown callback triggered by module")
        self.shutdown()


    def restart(self)-> None:
        for module in self.modules:
            try:
                self.shutdown()
                self.launch()
            except Exception as e:
                self.get_logger().error(f"Error during restart: {e}")
                self.state = ModuleState.Error
    
    def handle_module_restart(self):
        self.get_logger().info(" restart callback triggered by module")
        self.restart()

    def isReady(self) -> bool:
        '''
        Checks if all modules are ready
        '''
        self.get_logger().info(f"called isready and no. of modules is {len(self.modules)}")  # Print module count
        if not self.modules:
            self.get_logger().info("ERROR: self.modules is EMPTY! No modules have been added.")
            return False  
        
        for module in self.modules:

            self.get_logger().info(f"Module {module.pkg} state: {module.state}")
            if module.hasHeartbeat and module.state not in (ModuleState.Ready, ModuleState.Running):
            #if module.state not in (ModuleState.Ready, ModuleState.Running):  
                self.get_logger().info(f"Module {module.pkg} is not ready (current state: {module.state})")
                return False  # Not ready

        return True  # All modules are ready

"""
    def update(self) -> None:
        '''
        f
        Updates the visualizer for all modules along with adding extra information
        Current extra information:
        - Mission Type (static A, static B, etc)
        
        '''
        if not self.isLaunched or not self.visualizer:
            return

        extraText = [
            ["Mission Type", self.missionType, 2],
        ]

        states = ["starting", "ready", "RUNNING", "error", "shutdown", "unreponsive"]
"""
    #main

def main(args=None):
    # Ensure this script only runs when executed directly
    if __name__ == "__main__":
        rclpy.init(args=args)
        try:
            node = MissionLauncher()
            print("✅ Supervisor initialized.")
            rclpy.spin(node)
            print("⚠️ spin() returned — this should not happen unless node shut down.")
        except Exception as e:
            print(f"❌ Supervisor crashed with error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("🛑 Shutting down Supervisor.")
            #node.shutdown()
            #rclpy.shutdown()  #trying to revive spp