# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import json
from math import sqrt
import struct
import sys
from time import time

from bno055 import registers
from bno055.connectors.Connector import Connector
from bno055.params.NodeParameters import NodeParameters

from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String


class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node: Node, connector: Connector, param: NodeParameters):
        self.node = node
        self.con = connector
        self.param = param

        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)

        # create topic publishers:
        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', QoSProf)
        self.pub_imu = node.create_publisher(Imu, prefix + 'imu', QoSProf)
        self.pub_mag = node.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', QoSProf)
        self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', QoSProf)

    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info('Configuring device...')
        try:
            data = self.con.receive(registers.BNO055_CHIP_ID_ADDR, 1)
            if data[0] != registers.BNO055_ID:
                raise IOError('Device ID=%s is incorrect' % data)
            # print("device sent ", binascii.hexlify(data))
        except Exception as e:  # noqa: B902
            # This is the first communication - exit if it does not work
            self.node.get_logger().error('Communication error: %s' % e)
            self.node.get_logger().error('Shutting down ROS node...')
            sys.exit(1)

        # IMU connected => apply IMU Configuration:
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')

        if not (self.con.transmit(registers.BNO055_PWR_MODE_ADDR, 1, bytes([registers.POWER_MODE_NORMAL]))):
            self.node.get_logger().warn('Unable to set IMU normal power mode.')

        if not (self.con.transmit(registers.BNO055_PAGE_ID_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to set IMU register page 0.')

        if not (self.con.transmit(registers.BNO055_SYS_TRIGGER_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to start IMU.')

        if not (self.con.transmit(registers.BNO055_UNIT_SEL_ADDR, 1, bytes([0x83]))):
            self.node.get_logger().warn('Unable to set IMU units.')

        # The sensor placement configuration (Axis remapping) defines the
        # position and orientation of the sensor mount.
        # See also Bosch BNO055 datasheet section Axis Remap
        mount_positions = {
            'P0': bytes(b'\x21\x04'),
            'P1': bytes(b'\x24\x00'),
            'P2': bytes(b'\x24\x06'),
            'P3': bytes(b'\x21\x02'),
            'P4': bytes(b'\x24\x03'),
            'P5': bytes(b'\x21\x02'),
            'P6': bytes(b'\x21\x07'),
            'P7': bytes(b'\x24\x05')
        }
        if not (self.con.transmit(registers.BNO055_AXIS_MAP_CONFIG_ADDR, 2,
                mount_positions[self.param.placement_axis_remap.value])):
            self.node.get_logger().warn('Unable to set sensor placement configuration.')

        # Set Device to NDOF mode
        # data fusion for gyroscope, acceleration sensor and magnetometer enabled
        # absolute orientation
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')

        self.node.get_logger().info('Bosch BNO055 IMU configuration complete.')

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        # Initialize ROS msgs
        imu_raw_msg = Imu()
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        # read from sensor
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        # Publish raw data
        # TODO: convert rcl Clock time to ros time?
        # imu_raw_msg.header.stamp = node.get_clock().now()
        imu_raw_msg.header.frame_id = self.param.frame_id.value
        # TODO: do headers need sequence counters now?
        # imu_raw_msg.header.seq = seq

        # TODO: make this an option to publish?
        imu_raw_msg.orientation_covariance[0] = -1
        imu_raw_msg.linear_acceleration.x = float(
            struct.unpack('h', struct.pack('BB', buf[0], buf[1]))[0]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.y = float(
            struct.unpack('h', struct.pack('BB', buf[2], buf[3]))[0]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.z = float(
            struct.unpack('h', struct.pack('BB', buf[4], buf[5]))[0]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration_covariance[0] = -1
        imu_raw_msg.angular_velocity.x = float(
            struct.unpack('h', struct.pack('BB', buf[12], buf[13]))[0]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.y = float(
            struct.unpack('h', struct.pack('BB', buf[14], buf[15]))[0]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.z = float(
            struct.unpack('h', struct.pack('BB', buf[16], buf[17]))[0]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity_covariance[0] = -1
        # node.get_logger().info('Publishing imu message')
        self.pub_imu_raw.publish(imu_raw_msg)

        # TODO: make this an option to publish?
        # Publish filtered data
        # imu_msg.header.stamp = node.get_clock().now()
        imu_msg.header.frame_id = self.param.frame_id.value

        q = Quaternion()
        # imu_msg.header.seq = seq
        q.w = float(struct.unpack('h', struct.pack('BB', buf[24], buf[25]))[0])
        q.x = float(struct.unpack('h', struct.pack('BB', buf[26], buf[27]))[0])
        q.y = float(struct.unpack('h', struct.pack('BB', buf[28], buf[29]))[0])
        q.z = float(struct.unpack('h', struct.pack('BB', buf[30], buf[31]))[0])
        # TODO(flynneva): replace with standard normalize() function
        # normalize
        norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        imu_msg.orientation.x = q.x / norm
        imu_msg.orientation.y = q.y / norm
        imu_msg.orientation.z = q.z / norm
        imu_msg.orientation.w = q.w / norm

        imu_msg.linear_acceleration.x = float(
            struct.unpack('h', struct.pack('BB', buf[32], buf[33]))[0]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.y = float(
            struct.unpack('h', struct.pack('BB', buf[34], buf[35]))[0]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.z = float(
            struct.unpack('h', struct.pack('BB', buf[36], buf[37]))[0]) / self.param.acc_factor.value
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = float(
            struct.unpack('h', struct.pack('BB', buf[12], buf[13]))[0]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.y = float(
            struct.unpack('h', struct.pack('BB', buf[14], buf[15]))[0]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.z = float(
            struct.unpack('h', struct.pack('BB', buf[16], buf[17]))[0]) / self.param.gyr_factor.value
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data
        # mag_msg.header.stamp = node.get_clock().now()
        mag_msg.header.frame_id = self.param.frame_id.value
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = \
            float(struct.unpack('h', struct.pack('BB', buf[6], buf[7]))[0]) / self.param.mag_factor.value
        mag_msg.magnetic_field.y = \
            float(struct.unpack('h', struct.pack('BB', buf[8], buf[9]))[0]) / self.param.mag_factor.value
        mag_msg.magnetic_field.z = \
            float(struct.unpack('h', struct.pack('BB', buf[10], buf[11]))[0]) / self.param.mag_factor.value
        self.pub_mag.publish(mag_msg)

        # Publish temperature
        # temp_msg.header.stamp = node.get_clock().now()
        temp_msg.header.frame_id = self.param.frame_id.value
        # temp_msg.header.seq = seq
        temp_msg.temperature = float(buf[44])
        self.pub_temp.publish(temp_msg)

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        calib_status_str = String()
        calib_status_str.data = json.dumps(calib_status_dict)

        # Publish via ROS topic:
        self.pub_calib_status.publish(calib_status_str)

    def get_calib_offsets(self):
        """Read all calibration offsets and print to screen."""
        accel_offset_read = self.con.receive(registers.ACCEL_OFFSET_X_LSB_ADDR, 6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_offset_read = self.con.receive(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        gyro_offset_read = self.con.receive(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        self.node.get_logger().info(
            'Accel offsets (x y z): %d %d %d' % (
                accel_offset_read_x,
                accel_offset_read_y,
                accel_offset_read_z))

        self.node.get_logger().info(
            'Mag offsets (x y z): %d %d %d' % (
                mag_offset_read_x,
                mag_offset_read_y,
                mag_offset_read_z))

        self.node.get_logger().info(
            'Gyro offsets (x y z): %d %d %d' % (
                gyro_offset_read_x,
                gyro_offset_read_y,
                gyro_offset_read_z))

    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset):
        """
        Write calibration data (define as 16 bit signed hex).

        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        """
        # Must switch to config mode to write out
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().error('Unable to set IMU into config mode')
        time.sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.con.transmit(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset[0] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset[1] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset[2] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset[2] >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset[0] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset[1] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset[2] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset[2] >> 8) & 0xFF]))

            self.con.transmit(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset[0] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset[1] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset[2] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset[2] >> 8) & 0xFF]))
            return True
        except Exception:  # noqa: B902
            return False
