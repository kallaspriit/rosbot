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


from bno055 import registers
from bno055.error_handling.exceptions import BusOverRunException, TransmissionException
from rclpy.node import Node


class Connector:
    """
    Parent class for bno055 connectors.

    This class does NOT contain protocol-specific code for UART, I2C, etc.
    """

    def __init__(self, node: Node):
        self.node = node

    def receive(self, reg_addr, length):
        """
        Receives data packages of requested length from the sensor.

        :param reg_addr: The register address
        :param length: The length of the data package to receive
        :return: The received payload message
        :raises TransmissionException in case of any error
        """
        buf_out = bytearray()
        buf_out.append(registers.COM_START_BYTE_WR)
        buf_out.append(registers.COM_READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try:
            self.write(buf_out)
            buf_in = bytearray(self.read(2 + length))
            # print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except Exception as e:  # noqa: B902
            # re-raise as IOError
            raise TransmissionException('Transmission error: %s' % e)

        # Check for valid response length (the smallest (error) message has at least 2 bytes):
        if buf_in.__len__() < 2:
            raise TransmissionException('Unexpected length of READ-request response: %s'
                                        % buf_in.__len__())

        # Check for READ result (success or failure):
        if buf_in[0] == registers.COM_START_BYTE_ERROR_RESP:
            # Error 0x07 (BUS_OVER_RUN_ERROR) can be "normal" if data fusion is not yet ready
            if buf_in[1] == 7:
                # see #5
                raise BusOverRunException('Data fusion not ready, resend read request')
            else:
                raise TransmissionException('READ-request failed with error code %s'
                                            % hex(buf_in[1]))
        # Check for correct READ response header:
        if buf_in[0] != registers.COM_START_BYTE_RESP:
            raise TransmissionException('Wrong READ-request response header %s' % hex(buf_in[0]))

        if (buf_in.__len__()-2) != buf_in[1]:
            raise TransmissionException('Payload length mismatch detected: '
                                        + '  received=%s, awaited=%s'
                                        % (buf_in.__len__()-2, buf_in[1]))

        # Check for correct READ-request response length
        if buf_in.__len__() != (2 + length):
            raise TransmissionException('Incorrect READ-request response length: %s'
                                        % (2 + length))

        # remove the 0xBB:
        buf_in.pop(0)
        # remove the length information:
        buf_in.pop(0)

        # Return the received payload:
        return buf_in

    # -----------------------------
    def transmit(self, reg_addr, length, data: bytes):
        """
        Transmit data packages to the sensor.

        :param reg_addr: The register address
        :param length: The data length
        :param data: data to transmit
        :return:
        """
        buf_out = bytearray()
        buf_out.append(registers.COM_START_BYTE_WR)
        buf_out.append(registers.COM_WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        # Append payload data to the written:
        buf_out += data

        # print("Writing: ", binascii.hexlify(buf_out))

        try:
            self.write(buf_out)
            buf_in = bytearray(self.read(2))
            # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except Exception:  # noqa: B902
            return False

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            # rospy.logerr("Incorrect Bosh IMU device response.")
            return False
        return True
