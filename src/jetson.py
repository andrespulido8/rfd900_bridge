#!/usr/bin/env python
import rospy
import tf2_ros
import struct
import serial
import time


class RFD900_Rover:
    def __init__(self):
        port = "/dev/ttyUSB0"
        self.s = serial.Serial(port, 38400)
        rospy.init_node('rfd_rover', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.serial_buffer = ""

    def read_msg(self):
        current_read = ''
        current_read = self.s.read(self.s.inWaiting())
        self.serial_buffer = self.serial_buffer + current_read

        while self.serial_buffer.find('\x04\x17\xfe') > 0:
            data, self.serial_buffer = self.serial_buffer.split(
                '\x04\x17\xfe', 1)
            self.process_transform_data(data)

    def process_transform_data(self, data):
        tf_fmt = 'c10s10s7f'
        if len(data) == struct.calcsize(tf_fmt):
            msg_type, frame_id, child_frame_id, tx, ty, tz, rx, ry, rz, rw = struct.unpack(
                tf_fmt, data)
            rospy.loginfo("Received transform data:")
            rospy.loginfo("  Message type: %s", msg_type)
            rospy.loginfo("  Frame ID: %s", frame_id)
            rospy.loginfo("  Child Frame ID: %s", child_frame_id)
            rospy.loginfo("  Translation: (%f, %f, %f)", tx, ty, tz)
            rospy.loginfo("  Rotation: (%f, %f, %f, %f)", rx, ry, rz, rw)

    def read_msg_spinner(self):
        while not rospy.is_shutdown():
            try:
                if self.s.inWaiting() > 0:
                    self.read_msg()
            except IOError:
                rospy.logwarn("Serial IO Error")

    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()


if __name__ == '__main__':
    aa = RFD900_Rover()
    aa.spinner()
