from .mmwave_submodules.mmWaveClass import mmWaveSystem
import rclpy
from rclpy.node import Node
import threading
import time

from radar_interface.msg import Frame

class RadarNode(Node): 

    def __init__(self):
        super().__init__("radar")
        self.get_logger().info(":> Starting Radar Node")

        # setup device
        self.get_logger().info(":> Instantiating mmWaveSystem Object")
        self.radar = mmWaveSystem()

        self.get_logger().info(":> Setting Up DCA1000")
        self.radar.setup_DCA1000()

        self.get_logger().info(":> Configuring mmWave Device")
        self.radar.configure_mmWaveDevice() #needs to be done after power up and reset

        # set up radar publisher
        self.get_logger().info(":> Initializing Radar Publisher")
        self.radarPublisher = self.create_publisher(Frame, "frame_Data", 10)

        # setup threads
        self.collectThread = threading.Thread(target=self.collect, args=(),daemon=True)
        self.pubThread = threading.Thread(target=self.publishFrame, args=(),daemon=True)

        # start threads before recording starts so PC is ready
        self.get_logger().info(":> Starting Data Collection and Publishing Threads")
        self.collectThread.start()
        self.pubThread.start()

        # start the recording process
        self.get_logger().info(":> Starting mmWave Device Wave Transmission and ADC Recording")
        self.radar.start_Record()
    
    # thread functions
    def collect(self):
        while not self.radar.allFramesCollected:
            self.radar.collect_Data()

    def publishFrame(self):
        while not self.radar.allFramesCollected:
            msg = Frame() 
            frame = self.radar.get_Frame()
            if not frame == None:
                msg.frameSize = self.radar.frameSize
                msg.frameData = frame
                self.radarPublisher.publish(msg)
    
    def radarLog(self):
        while not self.radar.allFramesCollected:
            if not (self.radar.logString == None):
                if self.radar.logType == "Info":
                    self.get_logger().info(self.radar.logString)
                elif self.radar.logType == "Error":
                    self.get_logger().error(self.radar.logString)
                self.radar.logString = None

def main(args=None):
    rclpy.init()
    node = RadarNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
