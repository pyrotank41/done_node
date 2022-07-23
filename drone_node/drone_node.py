
# refered the following links for understanding the use of uORB, ROS2  and messages
# https://www.mathworks.com/help/ros/ug/control-a-simulated-uav-using-ros2-and-px4-bridge.html
# https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp


# imports -----------------------------------------------------------------------------------------------------------------------------------

# generic imports
from concurrent.futures import Executor
import time
from xmlrpc.client import Boolean

# ros pkg imports
import rclpy

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# message imports
from px4_msgs.msg import OffboardControlMode, VehicleStatus, Timesync, VehicleOdometry,TrajectorySetpoint,VehicleCommand, VehicleControlMode
from std_msgs.msg import String


# classes -------------------------------------------------------------------------------------------------------------------------------------
class DroneNode(Node):

    def __init__(self, rtps_cb_group, offboard_heartbeat_cbg, drone_cb_group):

        super().__init__('drone_node')


        # subscribers topic for info from UAV system
        self.status_sub = self.create_subscription(VehicleStatus, "/fmu/vehicle_status/out", 
                                                        self.status_sub_callback, 
                                                        qos_profile_sensor_data, 
                                                        callback_group=rtps_cb_group)

        self.time_sub   = self.create_subscription(Timesync, "/fmu/timesync/out",
                                                        self.time_sub_callback, 
                                                        qos_profile_sensor_data,
                                                        callback_group=rtps_cb_group)
        
        self.odom_sub   = self.create_subscription(VehicleOdometry, "/fmu/vehicle_odometry/out",
                                                        self.odom_sub_callback, 
                                                        qos_profile_sensor_data,
                                                        callback_group=rtps_cb_group)
                                            
        self.vehicle_control_mode_sub   = self.create_subscription(VehicleControlMode, "/fmu/vehicle_control_mode/out",
                                                        self.vehicle_control_mode_sub_callback, 
                                                        qos_profile_sensor_data,
                                                        callback_group=rtps_cb_group )

        # control communication
        self.control_mode_pub= self.create_publisher(OffboardControlMode, "/fmu/offboard_control_mode/in", 10, 
                                                    callback_group= offboard_heartbeat_cbg)

        self.setpoint_pub    = self.create_publisher(TrajectorySetpoint, "/fmu/trajectory_setpoint/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)

        self.cmd_pub         = self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)
        
        # our maiun publisher
        self.heartbeat_offboard_control = self.create_timer(0.05, self.offboard_heartbeat, callback_group = offboard_heartbeat_cbg)

        # creating message variables
        self.status = VehicleStatus()
        self.time = Timesync()
        self.odom = VehicleOdometry()
        self.vehicle_control_mode = VehicleControlMode()
        
        self.setpoint = (0.0, 0.0, 0.0)
        self.verbose = False
        self.heartbeat_num = 0
        
        
        self.main_loop = self.create_timer(1, self.main_callback_loop, callback_group=drone_cb_group)


    def run(self):
        while True:
            print(f"in run {self.status.arming_state}")
            time.sleep(1)

    # https://docs.px4.io/main/en/msg_docs/vehicle_status.html
    def status_sub_callback(self, msg:VehicleStatus):
        if self.verbose: print("status sub callback")
    
        if self.status.arming_state != msg.arming_state:
            print(f"armed state changed from {self.status.arming_state} to {msg.arming_state}")

        self.status = msg

    def offboard_heartbeat(self):
        self.heartbeat_num += 1
        self.publish_offboard_control_mode('position')
        self.publish_trejectory_setpoint_position(self.setpoint)



    def main_callback_loop(self):
        while self.time.timestamp == 0:

            print(".", end="")
            time.sleep(0.001)
        
        print("\n... Got initial vehicle status ...")
        print("... starting flow of control messages ...\n")
        time.sleep(1)
        
        # necessary:  offboard_control_mode needs to be paired with trajectory_setpoint
        while self.heartbeat_num < 10:
            time.sleep(1)

        self.engage_offboard_mode()
        self.arm()
        time.sleep(1)


        # takeoff
        self.takeoff_altitude = -20.0

        self.takeoff()

        time.sleep(3)

        self.land()

        time.sleep(10000)
        # self.arm(False)
        # self.timer.destroy()


    ## vehicle status call backs ---------------------------------------------------------------------------------------------

    def vehicle_control_mode_sub_callback(self, msg:VehicleControlMode):
        
        if self.vehicle_control_mode.flag_control_offboard_enabled != msg.flag_control_offboard_enabled:
            print(f"# vehicle offboard mode: {self.vehicle_control_mode.flag_control_offboard_enabled} \
                 to {msg.flag_control_offboard_enabled}")
            
        self.vehicle_control_mode = msg

    def time_sub_callback(self, msg:Timesync):
        if self.verbose: print("time_sub_callback")
        self.time = msg   
        

    def odom_sub_callback(self, msg:VehicleOdometry):
        if self.verbose: print("odom_sub_Callback")
        self.odom = msg
        pass


    ## vehicle control helper functions ----------------------------------------------------------------------------------------

    def engage_offboard_mode(self):
        # allow offboard control message to be utilized
        cmd_msg = VehicleCommand()

        cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE 
        cmd_msg.param1 = 1.0
        cmd_msg.param2 = 6.0

        # print("# Engaging offboard mode ...")
        self.publish_vehicle_cmd(cmd_msg) 


    def arm(self, state: Boolean = True ):
        # print("# Arming vehicle...")
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = float(state)
        self.publish_vehicle_cmd(cmd_msg)
        
        while self.status.arming_state != 2:
            print(".", end="")
            time.sleep(1)
        
        print("\n# Armed ....")

    def takeoff(self):

        self.setpoint = (0.0, 0.0, self.takeoff_altitude)
        while abs(self.odom.z) < abs(self.takeoff_altitude*0.95):
            time.sleep(0.5)
        
        print("reached takeoff altitude...")
        
        
    def land(self):
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publish_vehicle_cmd(cmd_msg)
        print("## Landing ...")
        while abs(self.odom.z) < 0.5:
            time.sleep(1)
        
        print("## ... Landed")


    def publish_offboard_control_mode(self, control_type:String):
        
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = self.time.timestamp

        # setting all the param to false
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False  

        # now selecting the mode according to the input
        if  control_type == 'position':
            mode_msg.position = True
        
        elif control_type == 'velocity':
            mode_msg.velocity = True
        
        elif control_type == 'acceleration':
            mode_msg.acceleration = True
        
        elif control_type == 'attitude':
            mode_msg.attitude = True

        self.control_mode_pub.publish(mode_msg)

    def publish_trejectory_setpoint_position(self,  point):
        #  used to send a setpoint position
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = self.time.timestamp
        setpoint_msg.position = point
        setpoint_msg.yaw = -3.14
        
        self.setpoint_pub.publish(setpoint_msg)

    def publish_vehicle_cmd(self, msg:VehicleCommand):

        msg.timestamp = self.time.timestamp
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component= 1
        msg.from_external= True
       
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()

    mutually_exclusive_cg_1 = MutuallyExclusiveCallbackGroup()
    mutually_exclusive_cg_2 = MutuallyExclusiveCallbackGroup()
    mutually_exclusive_cg_3 = MutuallyExclusiveCallbackGroup()

    reentramt_cb_group1 = ReentrantCallbackGroup()
    reentramt_cb_group2 = ReentrantCallbackGroup()
    
    node = DroneNode(mutually_exclusive_cg_1, mutually_exclusive_cg_2, mutually_exclusive_cg_3)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        print("")
        node.get_logger().info("Begining Drone Node, end with CTRL-C")
        executor.spin()
    
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down :)")

    node.destroy_node()
    rclpy.shutdown()