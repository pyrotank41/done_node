
# refered the following link for understanding the use of uORB, ROS2  and messages
# https://www.mathworks.com/help/ros/ug/control-a-simulated-uav-using-ros2-and-px4-bridge.html

from distutils import cmd
import threading
import time
import asyncio
from tokenize import String
import threading
from matplotlib.pyplot import connect
import mavsdk
from mavsdk import System


# ros pkgs impoort
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros2_aruco_interfaces.msg import ArucoMarkers

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from px4_msgs.msg import OffboardControlMode, VehicleStatus, Timesync, VehicleOdometry,TrajectorySetpoint,VehicleCommand, VehicleControlMode

class GimbleTrackerNode(Node):

    def __init__(self, cb_group, drone_cb_group):
        super().__init__('gimble_tracking_node')

        #declare and read parameters
        self.declare_parameter('aruco_markers_topic', '/drone/ros2_aruco/aruco_markers')
        aruco_markers_topic = self.get_parameter('aruco_markers_topic').get_parameter_value().string_value
        
        # Set up subscriptions
        self.create_subscription(ArucoMarkers, aruco_markers_topic, self.aruco_sub_callback,  qos_profile_sensor_data, callback_group= cb_group)
        self.status_sub = self.create_subscription(VehicleStatus, "fmu/vehicle_status/out", 
                                                        self.status_sub_callback, 
                                                        qos_profile_sensor_data, 
                                                        callback_group=cb_group)
        self.timer = self.create_timer(1, self.main_callback, callback_group=drone_cb_group)

        self.aruco = ArucoMarkers()
        self.status = VehicleStatus()

        print("creating pool")
        
        loop = asyncio.get_event_loop()

        print("running pool")
        loop.run_until_complete(self.run())

        print("dropped out of the drone event loop")

        
    
    def status_sub_callback(self, msg:VehicleStatus):
        print("status sub callback")
    
        if self.status.arming_state != msg.arming_state:
            print(f"armed state changed from {self.status.arming_state} to {msg.arming_state}")

        self.status = msg

    def main_callback(self):
        print("timer callback")
        pass
        

    def aruco_sub_callback(self, msg:ArucoMarkers):
        print("got aruco message")
        self.aruco = msg

    async def run(self):
        print("here")
        self.drone = System()

        print("reached run")
        await self.drone.connect(system_address="udp://:14540")
        
        status_text_task = asyncio.ensure_future(self.print_status_text())

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        print("-- Arming")
        await self.drone.action.arm()

        print("-- Taking off")
        await self.drone.action.takeoff()

        await asyncio.sleep(10)

        print("-- Landing")
        await self.drone.action.land()

        status_text_task.cancel()

    async def print_status_text(self):
        try:
            async for status_text in self.drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return
        

def main():
    rclpy.init()
    

    mutually_exclusive_cg_1 = MutuallyExclusiveCallbackGroup()
    mutually_exclusive_cg_2 = MutuallyExclusiveCallbackGroup()

    reentramt_cb_group1 = ReentrantCallbackGroup()
    reentramt_cb_group2 = ReentrantCallbackGroup()
    
    node = GimbleTrackerNode(cb_group=mutually_exclusive_cg_1, drone_cb_group=mutually_exclusive_cg_2)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        print("")
        node.get_logger().info("Begining Gimble Tracker Node, end with CTRL-C")
        executor.spin()
    
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down :)")

    node.destroy_node()
    rclpy.shutdown()