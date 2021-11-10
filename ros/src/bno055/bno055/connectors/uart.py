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


# Connector for UART integration of the BNO-055
# See also https://pyserial.readthedocs.io/en/latest/pyserial_api.html
import sys

from bno055.connectors.Connector import Connector
from rclpy.node import Node
import serial


class UART(Connector):
    """Connector implementation for serial UART connection to the sensor."""

    CONNECTIONTYPE_UART = 'uart'

    def __init__(self, node: Node, baudrate, port, timeout):
        # Initialize parent class
        super().__init__(node)

        self.node = node
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.serialConnection = None

    def connect(self):
        self.node.get_logger().info('Opening serial port: "%s"...' % self.port)

        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.serialutil.SerialException:
            self.node.get_logger().info('Unable to connect to IMU at port ' + self.port)
            self.node.get_logger().info('Check to make sure your device is connected')
            sys.exit(1)

    def read(self, numberOfBytes):
        return self.serialConnection.read(numberOfBytes)

    def write(self, data: bytearray):
        self.serialConnection.write(data)
