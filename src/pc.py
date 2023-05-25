#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import serial
import struct
import time


class RFD900_GCS:
    def __init__(self):
        port = "/dev/ttyUSB0"
        self.s = serial.Serial(port, 38400)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        rospy.Subscriber("/tf", TFMessage, self.send_transform)
        self.serial_buffer = ""
        self.read_buffer = list()
        self.tf_fmt = 'c10s10s7f'

    def send_transform(self, tf_msg):
        for transform in tf_msg.transforms:
            t = TransformStamped()
            t.header = transform.header
            t.child_frame_id = transform.child_frame_id
            t.transform = transform.transform
            tfm = TFMessage([t])
            self.publish_tf(tfm)

    def publish_tf(self, tf_msg):
        for transform in tf_msg.transforms:
            read_msg = struct.pack(self.tf_fmt, 't', transform.header.frame_id.encode(),
                                   transform.child_frame_id.encode(),
                                   transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z,
                                   transform.transform.rotation.x,
                                   transform.transform.rotation.y,
                                   transform.transform.rotation.z,
                                   transform.transform.rotation.w)
            self.s.write(read_msg)

    def read_msg(self):
        current_read = ''
        current_read = self.s.read(self.s.in_waiting)
        self.serial_buffer = self.serial_buffer + \
            current_read.decode(errors='ignore')
        self.serial_buffer = self.serial_buffer + current_read.decode()
        while self.serial_buffer.find(b'\x04\x17\xfe') > 0:
            data, self.serial_buffer = self.serial_buffer.split(
                b'\x04\x17\xfe', 1)
            self.read_buffer.append(data)
            print(data)

    def process_msgs(self):
        if len(self.read_buffer) > 0:
            msg = self.read_buffer.pop(0)
            msg_type = msg[0]
            try:
                # Process other message types here if needed
                pass
            except AttributeError:
                rospy.logwarn("AttributeError")
            except struct.error:
                rospy.logwarn("Corrupt Packet")

    def read_msg_spinner(self):
        while not rospy.is_shutdown():
            if self.s.inWaiting() > 0:
                self.read_msg()
            self.process_msgs()

    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()


aa = RFD900_GCS()
aa.spinner()
