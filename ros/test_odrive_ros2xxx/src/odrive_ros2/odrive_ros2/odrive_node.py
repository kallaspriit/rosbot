import sys
from math import pi

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl

import odrive
from odrive.enums import *


class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        self.declare_parameter('connection.timeout', 15, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description='ODrive connection timeout in seconds'))
        self.declare_parameter('battery.max_voltage', 4.2 * 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Max battery voltage'))
        self.declare_parameter('battery.min_voltage', 3.2 * 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Min battery voltage'))
        self.declare_parameter('battery.topic', 'barrery_percentage', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description='Battery percentage publisher topic'))
        self.declare_parameter('joint_state.topic', 'joint_state', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description='Joint State publisher topic'))

        self.connect_odrive_service = self.create_service(
            Trigger,
            'connect_odrive',
            self.connect_odrive_callback
        )

        self.request_state_service = self.create_service(
            AxisState,
            'request_state',
            self.request_state_callback
        )

        self.position_cmd_service = self.create_service(
            PositionControl,
            'position_cmd',
            self.position_cmd_callback
        )

        self.velocity_cmd_service = self.create_service(
            VelocityControl,
            'velocity_cmd',
            self.velocity_cmd_callback
        )

        self.battery_percentage_publisher_ = self.create_publisher(
            Float32,
            self.get_parameter(
                'battery.topic').get_parameter_value().string_value,
            1
        )
        self.battery_percentage_publisher_timer = self.create_timer(
            10,
            self.battery_percentage_publisher_callback
        )

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            self.get_parameter(
                'joint_state.topic').get_parameter_value().string_value,
            10
        )
        self.joint_state_publisher_timer = self.create_timer(
            0.1,
            self.joint_state_publisher_callback
        )

        self.driver: odrive.fibre.remote_object.RemoteObject = None

    def is_driver_ready(self):
        if self.driver:
            try:
                if self.driver.user_config_loaded:
                    return True
                else:
                    self.get_logger().warn('ODrive user config not loaded')
                    return False
            except:
                self.get_logger().error('Unexpected error:', sys.exc_info()[0])
                return False
        else:
            self.get_logger().debug('ODrive not connected')
            return False

    """
    AXIS_STATE_UNDEFINED = 0
    AXIS_STATE_IDLE = 1
    AXIS_STATE_STARTUP_SEQUENCE = 2
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_SENSORLESS_CONTROL = 5
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    AXIS_STATE_LOCKIN_SPIN = 9
    AXIS_STATE_ENCODER_DIR_FIND = 10
    """
    def request_state_callback(self, request: AxisState.Request, response: AxisState.Response):
        if self.is_driver_ready():
            if request.axis == 0:
                self.driver.axis0.requested_state = request.state
                self.driver.axis0.watchdog_feed()
                response.success = True
                response.state = self.driver.axis0.current_state
                response.message = f'Success'
            elif request.axis == 1:
                self.driver.axis1.requested_state = request.state
                self.driver.axis1.watchdog_feed()
                response.success = True
                response.state = self.driver.axis0.current_state
                response.message = f'Success'
            else:
                response.success = False
                response.message = f'Axis not exist'
        else:
            response.success = False
            response.message = f'ODrive not ready'
        
        return response

    def position_cmd_callback(self, request: PositionControl.Request, response: PositionControl.Response):
        if self.is_driver_ready():
            if request.axis == 0:
                self.driver.axis0.controller.input_pos = request.turns
                self.driver.axis0.watchdog_feed()
                response.success = True
                response.message = f'Success'
            elif request.axis == 1:
                self.driver.axis1.controller.input_pos = request.turns
                self.driver.axis1.watchdog_feed()
                response.success = True
                response.message = f'Success'
            else:
                response.success = False
                response.message = f'Axis not exist'
        else:
            response.success = False
            response.message = f'ODrive not ready'
        
        return response

    def velocity_cmd_callback(self, request: VelocityControl.Request, response: VelocityControl.Response):
        if self.is_driver_ready():
            if request.axis == 0:
                self.driver.axis0.controller.input_vel = request.turns_s
                self.driver.axis0.watchdog_feed()
                response.success = True
                response.message = f'Success'
            elif request.axis == 1:
                self.driver.axis1.controller.input_vel = request.turns_s
                self.driver.axis1.watchdog_feed()
                response.success = True
                response.message = f'Success'
            else:
                response.success = False
                response.message = f'Axis not exist'
        else:
            response.success = False
            response.message = f'ODrive not ready'
        
        return response

    def connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self.get_logger().info('Connecting to ODrive')
            self.driver = odrive.find_any(
                timeout=self.get_parameter(
                    'connection.timeout'
                ).get_parameter_value().integer_value)
            self.get_logger().info('ODrive connected')
            response.success = True
            response.message = f'Connected to {self.driver.serial_number}'
            if not self.driver.user_config_loaded:
                self.get_logger().warn('ODrive user config not loaded')
            self.driver.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.driver.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except TimeoutError:
            response.success = False
            response.message = 'Timeout'
        except:
            response.success = False
            response.message = 'Unexpected error:', sys.exc_info()[0]

        return response

    def battery_percentage_publisher_callback(self):
        if self.is_driver_ready():
            msg = Float32()
            msg.data = (
                self.driver.vbus_voltage -
                self.get_parameter(
                    'battery.min_voltage').get_parameter_value().double_value
            ) / (
                self.get_parameter('battery.max_voltage').get_parameter_value().double_value -
                self.get_parameter('battery.min_voltage').get_parameter_value().double_value)
            self.battery_percentage_publisher_.publish(msg)
            if msg.data < 0.2:
                self.get_logger().warn(
                    f'ODrive battery percentage low: {msg.data:0.2f}')
        else:
            self.get_logger().debug('ODrive not ready')

    def joint_state_publisher_callback(self):
        if self.is_driver_ready():
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [self.driver.axis0.encoder.pos_estimate,
                            self.driver.axis1.encoder.pos_estimate]
            msg.velocity = [self.driver.axis0.encoder.vel_estimate,
                            self.driver.axis1.encoder.vel_estimate]
            self.joint_state_publisher_.publish(msg)
        else:
            self.get_logger().debug('ODrive not ready')


def main():
    print('Hi from odrive_ros2.')
    rclpy.init()
    node = ODriveNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
