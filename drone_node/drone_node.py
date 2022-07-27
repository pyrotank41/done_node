
# refered the following links for understanding the use of uORB, ROS2  and messages
# https://www.mathworks.com/help/ros/ug/control-a-simulated-uav-using-ros2-and-px4-bridge.html
# https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp


# imports -----------------------------------------------------------------------------------------------------------------------------------

# generic imports
import time
import math

# ros pkg imports
import rclpy

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# message imports
from px4_msgs.msg import OffboardControlMode, VehicleStatus, Timesync, VehicleOdometry, \
                            TrajectorySetpoint, VehicleCommand, VehicleControlMode, LandingGear, \
                            GimbalManagerSetManualControl, GimbalDeviceAttitudeStatus
from ros2_aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import String


#import temp packages
import pygame



# classes -------------------------------------------------------------------------------------------------------------------------------------
class DroneNode(Node):

    def __init__(self, rtps_cb_group, offboard_heartbeat_cbg, drone_cb_group):

        super().__init__('drone_node')

        self.drone_cbg = drone_cb_group

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

        self.gimbal_device_atiude_status   = self.create_subscription(GimbalDeviceAttitudeStatus, "/fmu/gimbal_device_attitude_status/out",
                                                        self.gimbal_device_atiude_status_cb, 
                                                        2,
                                                        callback_group=rtps_cb_group )
        

        # control communication
        self.control_mode_pub= self.create_publisher(OffboardControlMode, "/fmu/offboard_control_mode/in", 10, 
                                                    callback_group= offboard_heartbeat_cbg)

        self.setpoint_pub    = self.create_publisher(TrajectorySetpoint, "/fmu/trajectory_setpoint/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)

        self.cmd_pub         = self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)

        self.landing_gear = self.create_publisher(LandingGear, "/fmu/landing_gear/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)

        self.gimbal_manager_set_manual_control = self.create_publisher(GimbalManagerSetManualControl, "/fmu/gimbal_manager_set_manual_control/in", 10,
                                                    callback_group= offboard_heartbeat_cbg)


        
        
        # our main publisher to keep the connection alive between offboard and flight computer
        self.heartbeat_offboard_control = self.create_timer(0.05, self.offboard_heartbeat, callback_group = offboard_heartbeat_cbg)

        # creating message variables, callbacks will update these variables
        self.status = VehicleStatus()
        self.time = Timesync()
        self.odom = VehicleOdometry()
        self.vehicle_control_mode = VehicleControlMode()
        

        self.setpoint = (0.0, 0.0, 0.0)
        self.verbose = False
        self.heartbeat_num = 0
        self.takeoff_altitude = -3.0

        self.control_mode = 'position'
        self.pitch = float(-45) #deg
        self.yaw = float(0)     #deg


        pygame.init()
        self.window = pygame.display.set_mode((300, 300))
        
        self.main_loop = self.create_timer(0.01, self.main_callback_loop, callback_group=drone_cb_group)

    # calback function to keep connection with the flight controller alive
    # this callback is running on a different calback group to avoid deadlocks and delays
    
    def offboard_heartbeat(self):

        # TODO: for some reason the drone is not disarming after landing, could the heartbeat be a problem?
        self.heartbeat_num += 1
        self.publish_offboard_control_mode(self.control_mode)
        self.publish_trejectory_setpoint_position(self.setpoint)


    def main_callback_loop(self):
        # testing features using keyboard input
        self.pygame_control()

    def yaw_pitch_check(self, yaw, pitch):
        if yaw >= 360:
            yaw = yaw - 360
            
        if  yaw <= 0:
            yaw = 360 - yaw  

        if pitch <= -90:
            pitch = -89.9
        if pitch >= 45:
            pitch = 45

        return yaw, pitch

    
    def pygame_control(self):
        # While being in air and landing mode the drone is not likely to takeoff again, so
        # a condition check is required here to avoid such a condition.

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t and event.key == pygame.K_LCTRL:
                    print("ctrl + t pressed")
                if event.key == pygame.K_l:
                    print("l pressed")
        
        keys = pygame.key.get_pressed()

        if keys[pygame.K_t] and keys[pygame.K_LCTRL]:
            # arm and takeoff
            print('ctrl+t Pressed')
            self.engage_offboard_mode()
            self.arm()
            self.takeoff()
            time.sleep(0.5)
        
        if keys[pygame.K_l] and keys[pygame.K_LCTRL]:
            # land at current position
            print("ctrl +l Pressed")
            self.land()
            time.sleep(0.5)

        # gimbal control
        if keys[pygame.K_a] or keys[pygame.K_s] or keys[pygame.K_d] or keys[pygame.K_w]: 
            
            pitch, yaw = self.pitch, self.yaw
            step = 1

            if keys[pygame.K_d]:
                yaw, pitch = self.yaw_pitch_check(yaw + step, pitch)   

            if keys[pygame.K_a]:
                yaw, pitch = self.yaw_pitch_check(yaw - step, pitch)

            if keys[pygame.K_w]:
                yaw, pitch = self.yaw_pitch_check(yaw, pitch + step)

            if keys[pygame.K_s]:
                yaw, pitch = self.yaw_pitch_check(yaw, pitch - step)

            self.pitch, self.yaw = pitch, yaw
            self.publish_gimbal_cmd(pitch, yaw)
            time.sleep(0.1)

            print(f"pitch: {self.pitch}, yaw: {self.yaw}")
            print(f"gpitch: {self.gimbal_pitch}, gyaw: {self.gimbal_yaw}")


        if keys[pygame.K_f]:
            self.get_logger().info("aruco_tracking_mode")
            self.aruco_track()
            time.sleep(1)

    def aruco_track(self):
        self.aruco_sub = self.create_subscription(ArucoMarkers, "/drone/ros2_aruco/aruco_markers",
                                                        self.aruco_sub_cb, 
                                                        qos_profile_sensor_data,
                                                        callback_group=self.drone_cbg)
        
        pass

    def get_desired_gimble_angles(self, x, y, distance_2d_x, distance_2d_y):
        gimbal_yaw_error = math.degrees(math.atan(x/distance_2d_x)) # in deg
        gimbal_pitch_error = math.degrees(math.atan(y/distance_2d_y)) # in deg
       
        yaw = self.gimbal_yaw + gimbal_yaw_error
        pitch = self.gimbal_pitch - gimbal_pitch_error

        yaw, pitch = self.yaw_pitch_check(yaw, pitch)
        print(gimbal_pitch_error, self.gimbal_pitch, pitch)

        return yaw, pitch

    def aruco_sub_cb(self, msg):
        # get current position the marker at 0th index
        position = msg.poses[0].position

        #  
        aruco_height = -2.25 #Meters
        z = self.odom.z - aruco_height

        distance_2d_x = math.sqrt(position.x**2 + position.z**2)
        distance_2d_y = math.sqrt(position.y**2 + position.z**2)

        yaw, pitch = self.get_desired_gimble_angles(position.x, position.y, distance_2d_x, distance_2d_y)

        
        self.publish_gimbal_cmd(pitch, yaw)


        # print(position.x , position.y, position.z)
        pass

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in deg (counterclockwise)
        pitch is rotation around y in deg (counterclockwise)
        yaw is rotation around z in deg (counterclockwise)

        TODO: this conversion function is tested for yaw and pitch but not yet for roll
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        yaw_z =  (math.degrees(math.atan2(t0, t1)) -180) * -1
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = -1 * math.degrees(math.asin(t2))
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        roll_x = math.degrees(math.atan2(t3, t4))
     
        return roll_x, pitch_y, yaw_z # in degrees


    ## vehicle status call backs ---------------------------------------------------------------------------------------------

    def gimbal_device_atiude_status_cb(self, msg:GimbalDeviceAttitudeStatus):
        q = msg.q
        self.gimbal_roll, self.gimbal_pitch, self.gimbal_yaw = self.euler_from_quaternion(q[0], q[1], q[2], q[3])

        
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

    # https://docs.px4.io/main/en/msg_docs/vehicle_status.html
    def status_sub_callback(self, msg:VehicleStatus):
        if self.verbose: print("status sub callback")
    
        if self.status.arming_state != msg.arming_state:
            print(f"armed state changed from {self.status.arming_state} to {msg.arming_state}")

        self.status = msg


    ## vehicle control helper functions ----------------------------------------------------------------------------------------

    def engage_offboard_mode(self):
        # allow offboard control message to be utilized
        self.get_logger().info("## Engaging offboard mode ...")
        cmd_msg = VehicleCommand()

        cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE 
        cmd_msg.param1 = 1.0
        cmd_msg.param2 = 6.0

        
        self.publish_vehicle_cmd(cmd_msg)

        # TODO: add a time out here, in case the drone is not ready and the command was issued, this will reasult in a dead lock
        while self.vehicle_control_mode.flag_control_offboard_enabled is False:
            time.sleep(1)
        
        self.get_logger().info("... Engaged offboard mode")
        
        time.sleep(1) 

    def arm(self, state=True ):

        self.get_logger().info("## Arming vehicle...")
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = float(state)
        self.publish_vehicle_cmd(cmd_msg)
        
        while self.status.arming_state != 2:
            time.sleep(1)
        
        self.get_logger().info("## Armed ....")

    def takeoff(self):

        self.get_logger().info("## Taking off")
        self.setpoint = (0.0, 0.0, self.takeoff_altitude)
        while abs(self.odom.z) < abs(self.takeoff_altitude*0.95):
            time.sleep(0.5)
        
        self.set_landing_gear_state(True)
        
        self.get_logger().info("Reached takeoff altitude...")

    def set_landing_gear_state(self, state):
        landing_gear_msg = LandingGear()
        landing_gear_msg.timestamp = self.time.timestamp
        landing_gear_msg.landing_gear = state
        self.landing_gear.publish(landing_gear_msg)
        
    def land(self):

        print("## Landed ...")
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publish_vehicle_cmd(cmd_msg) 

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
    
    def convert_deg_to_rad(self, pitch_deg:float, yaw_deg:float):
        pitch_rad = (math.pi/180)*pitch_deg
        yaw_rad = (math.pi/180)*yaw_deg
        return pitch_rad, yaw_rad

    def publish_gimbal_cmd(self, pitch:float, yaw:float):
        pitch, yaw = self.convert_deg_to_rad(pitch, yaw)
        gimbal_msg = GimbalManagerSetManualControl()
        gimbal_msg.origin_sysid = 0
        gimbal_msg.origin_compid = 0
        gimbal_msg.target_system = 0
        gimbal_msg.target_component = 0
        gimbal_msg.flags = 12
        gimbal_msg.gimbal_device_id = 0
        gimbal_msg.pitch = pitch
        gimbal_msg.yaw = yaw
        gimbal_msg.pitch_rate = 0.785398 # 60 deg/sec
        gimbal_msg.yaw_rate = 0.785398 # 60 deg/sec 
        self.gimbal_manager_set_manual_control.publish(gimbal_msg)



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
    
    except Exception as e:
        node.get_logger().error(e)

    node.destroy_node()
    rclpy.shutdown()