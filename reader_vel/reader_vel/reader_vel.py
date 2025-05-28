import numpy as np
import threading
import time
import os
import sys
import rclpy

from geometry_msgs.msg import Twist
from ppadb.client import Client as AdbClient
from rclpy.node import Node

class Reader(Node):
    def __init__(self,
            ip_address=None,
            port = 5555,
            print_FPS=False,
            run=True
        ):
        super().__init__('reader_vel')
        self.publisher_right = self.create_publisher(Twist, 'q2r_right_hand_twist', 10)
        self.publisher_left = self.create_publisher(Twist, 'q2r_left_hand_twist', 10)


        self.running = False
        self.tag = 'allHandVel'
        self.ip_address = ip_address
        self.port = port
        self.device = self.get_device()
        if run:
            self.run()

    def __del__(self):
        self.stop()

    def run(self):
        self.running = True
        # self.device.shell('am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER -f 0x10200000')
        self.thread = threading.Thread(target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line))
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()

    def get_network_device(self, client, retry=0):
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system('adb devices')
            client.remote_connect(self.ip_address, self.port)
        device = client.device(self.ip_address + ':' + str(self.port))

        if device is None:
            if retry==1:
                os.system('adb tcpip ' + str(self.port))
            if retry==2:
                eprint('Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`.')
                eprint('Currently provided IP address:', self.ip_address)
                eprint('Run `adb shell ip route` to verify the IP address.')
                exit(1)
            else:
                self.get_device(client=client, retry=retry+1)
        return device

    def get_usb_device(self, client):
        try:
            devices = client.devices()
        except RuntimeError:
            os.system('adb devices')
            devices = client.devices()
        for device in devices:
            if device.serial.count('.') < 3:
                return device
        eprint('Device not found. Make sure that device is running and is connected over USB')
        eprint('Run `adb devices` to verify that the device is visible.')
        exit(1)

    def get_device(self):
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    @staticmethod
    def process_data(string):
        print(string)
        try:
            strLeftVel, strRightVel = string.split('|')
        except ValueError:
            return None, None
        floatleftVel  = list(map(float, strLeftVel.strip().split()))
        floatrightVel = list(map(float, strRightVel.strip().split()))
        
        return floatleftVel, floatrightVel

    def extract_data(self, line):
        output = ''
        if self.tag in line:
            try:
                output += line.split(self.tag + ': ')[1]
            except ValueError:
                pass
        return output

    def create_twist(self, vel):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vel[0:3]
        twist.angular.x, twist.angular.y, twist.angular.z = vel[3:6]
        return twist

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    leftVel, rightVel = Reader.process_data(data)
                    if leftVel and rightVel:
                        left_twist = self.create_twist(leftVel)
                        right_twist = self.create_twist(rightVel)
                        self.publisher_left.publish(left_twist)
                        self.publisher_right.publish(right_twist)
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()

def main(args = None):
    rclpy.init(args=args)
    reader = Reader()
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        reader.stop()
    finally:
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main();