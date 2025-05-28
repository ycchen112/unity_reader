import numpy as np
import threading
import time
import os
import sys
import json
import rclpy

from ppadb.client import Client as AdbClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3
from handdata_msg.msg import OVR2ROSinputs

class Reader(Node):
    def __init__(self,
            ip_address=None,
            port = 5555,
            run=True
        ):
        super().__init__('reader_adb_json')
        self.pub_right_vel     = self.create_publisher(Twist, 'q2r_right_hand_twist', 10)
        self.pub_left_vel      = self.create_publisher(Twist, 'q2r_left_hand_twist', 10)
        self.pub_right_pose    = self.create_publisher(Pose, 'q2r_right_hand_pose', 10)
        self.pub_left_pose     = self.create_publisher(Pose, 'q2r_left_hand_pose', 10)
        self.pub_right_inputs  = self.create_publisher(OVR2ROSinputs, 'q2r_right_hand_inputs', 10)
        self.pub_left_inputs   = self.create_publisher(OVR2ROSinputs, 'q2r_left_hand_inputs', 10)    

        self.running = False
        self.tag = 'handDataJson'
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
                # eprint('Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`.')
                # eprint('Currently provided IP address:', self.ip_address)
                # eprint('Run `adb shell ip route` to verify the IP address.')
                print('Run `adb shell ip route` to verify the IP address.', file=sys.stderr)
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
            
        print('Device not found. Make sure that device is running and is connected over USB', file=sys.stderr)
        print('Run `adb devices` to verify that the device is visible.', file=sys.stderr)
        # eprint('Device not found. Make sure that device is running and is connected over USB')
        # eprint('Run `adb devices` to verify that the device is visible.')
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
        try:
            data = json.loads(string)
        except ValueError:
            return None
        
        return data

    def extract_data(self, line):
        output = ''
        if self.tag in line:
            try:
                output += line.split(self.tag + ': ')[1]
            except ValueError:
                pass
        return output

    def create_twist(self, lineVel, angularVel):
        twist = Twist()
        vec = Vector3()
        ang = Vector3()
        vec.x, vec.y, vec.z = lineVel
        ang.x, ang.y, ang.z = angularVel
        twist.linear  = vec
        twist.angular = ang
        return twist

    def create_pose(self, position, rotation):
        pose = Pose()
        pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose.orientation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
        return pose

    def create_inputs(self, buttons, triggers, thumbsticks):
        Rinputs = OVR2ROSinputs()
        Linputs = OVR2ROSinputs()

        Rinputs.button_upper = bool(buttons["A"])
        Rinputs.button_lower = bool(buttons["B"])
        Rinputs.thumb_stick = bool(buttons["RTS"])
        Rinputs.thumb_stick_horizontal = thumbsticks["RT"][0]
        Rinputs.thumb_stick_vertical = thumbsticks["RT"][1]
        Rinputs.press_index = triggers["RI"]
        Rinputs.press_middle = triggers["RH"]
        
        Linputs.button_upper = bool(buttons["X"])
        Linputs.button_lower = bool(buttons["Y"])
        Linputs.thumb_stick = bool(buttons["LTS"])
        Linputs.thumb_stick_horizontal = thumbsticks["LT"][0]
        Linputs.thumb_stick_vertical = thumbsticks["LT"][1]
        Linputs.press_index = triggers["LI"]
        Linputs.press_middle = triggers["LH"]

        return Rinputs, Linputs
        

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    # print(data)
                    data_loads = Reader.process_data(data)
                    self.show_data(data_loads)
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


    def show_data(self, data):
        # print("---- HAND DATA ----")
        rh_pos = data["rightHandPose"]["position"]
        lh_pos = data["leftHandPose"]["position"]
        rh_ori = data["rightHandPose"]["rotation"]
        lh_ori = data["leftHandPose"]["rotation"]
        rh_vel = data["rightHandPose"]["lineVel"]
        lh_vel = data["leftHandPose"]["lineVel"]
        rh_angvel = data["rightHandPose"]["angularVel"]
        lh_angvel = data["leftHandPose"]["angularVel"]
        buttons = data["buttons"]
        triggers = data["triggers"]
        thumbstickState = data["thumbsticks"]

        # print(f"Right Hand Pos: {rh_pos}")
        # print(f"Left  Hand Pos: {lh_pos}")
        # print(f"Right lineVel:{rh_vel}, angularVel:{rh_angvel}")
        # print(f"Left lineVel: {lh_vel}, angularVel:{lh_angvel}")
        # print(f"Buttons: A={buttons['A']}, B={buttons['B']}, X={buttons['X']}, Y={buttons['Y']}, RTS={buttons['RTS']}, LTS={buttons['LTS']}")
        # print(f"Triggers: RI={triggers['RI']}, RH={triggers['RH']}, LI={triggers['LI']}, LH={triggers['LH']}")
        # print(f"ThumbstickState: RT={[thumbstickState['RT']]}, LT={[thumbstickState['LT']]}")
        # print("-------------------\n")

        left_twist  = self.create_twist(lh_vel, lh_angvel)
        right_twist = self.create_twist(rh_vel, rh_angvel)
        left_pose   = self.create_pose(lh_pos, lh_ori)
        right_pose  = self.create_pose(rh_pos, rh_ori)
        right_inputs, left_inputs = self.create_inputs(buttons, triggers, thumbstickState)
        self.pub_left_inputs.publish(left_inputs)
        self.pub_left_pose.publish(left_pose)
        self.pub_left_vel.publish(left_twist)
        self.pub_right_inputs.publish(right_inputs)
        self.pub_right_pose.publish(right_pose)
        self.pub_right_vel.publish(right_twist)

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