import numpy as np
import csv 
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rclpy
import rclpy.time

class CSVtopath(Node):
    def __init__(self):
        super().__init__('csv_to_path')   
        self.publisher_=self.create_publisher(Path,'csvtopath',10)
        arr = np.genfromtxt('/home/fsai/wapoints_ws/src/path_csv/ipg_track_newww.csv', delimiter=',' )
        timer = self.create_timer(0.1, self.publishPath)
        self.pathMsg = Path()
        self.pathMsg.header.frame_id="map"
        self.rate = 10

        for idx,point in enumerate(arr):
            if (idx % self.rate==0):
                pose = PoseStamped()   
                pose.header.frame_id = "map"
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                self.pathMsg.poses.append(pose)


    def publishPath(self):
        self.publisher_.publish(self.pathMsg)

def main(args=None):
    rclpy.init(args=args)
    csvtopath =CSVtopath()
    csv_file='/home/fsai/wapoints_ws/src/path_csv/ipg_track_newww.csv'
    rclpy.spin(csvtopath)
    csv.path.destroy_node()
    rclpy.shutdown()
