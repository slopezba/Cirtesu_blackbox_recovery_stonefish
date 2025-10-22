#!/usr/bin/env python
import rospy

import socket
import struct
import json
import time 

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from stonefish_ros.msg import INS

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np


class Patch:
    def __init__(self):
        rospy.init_node('ardusim_patch', anonymous=True)

        self.namespace = rospy.get_namespace().strip('/')

        # Subscribers
        rospy.Subscriber("/bluerov/navigator/imu", INS, self._imu_callback)
        rospy.Subscriber("/bluerov/navigator/gps", NavSatFix, self._gps_callback)
        rospy.Subscriber("/bluerov/navigator/odometry", Odometry, self._odom_callback)

        # Publishers
        self.pub_pwm = rospy.Publisher("/bluerov/controller/thruster_setpoints_sim", Float64MultiArray, queue_size=1)

        self.sock_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_sitl.bind(('', 9002))
        self.sock_sitl.settimeout(0.1)

        self.imu = None
        self.gps = None
        self.odom = None
        self.prev_vel_body = None
        self.prev_time_imu = None

        self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
        self.gps_addr = ("127.0.0.1", 5760)

        self.timer = rospy.Timer(rospy.Duration(1.0 / 50), self.looper)
    
    def _imu_callback(self, msg):
        self.imu = msg

    def _gps_callback(self, msg):
        self.gps = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def looper(self, event):
        if self.odom is None:
            rospy.loginfo_throttle(5, "Waiting for IMU and Odometry callbacks...")
            return

        try:
            data, address = self.sock_sitl.recvfrom(100)
        except Exception:
            rospy.loginfo_throttle(5, "Socket receive failed, is SITL running?")
            return 

        parse_format = 'HHI16H'
        magic = 18458

        if len(data) != struct.calcsize(parse_format):
            rospy.logwarn("Got packet of unexpected length: %u", len(data))
            return 

        decoded = struct.unpack(parse_format, data)

        if magic != decoded[0]:
            rospy.logwarn("Incorrect protocol magic %u should be %u", decoded[0], magic)
            return 
        
        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        pwm = decoded[3:]

        pwm_thrusters = pwm[0:8]
        pwm_setpoint = [(x - 1500) / 400.0 for x in pwm[0:8]]
                
        # print(pwm_setpoint)

        # print([pwm[2], pwm[0]])
        # print("{:.2f} {:.2f}".format(pwm_setpoint[0], pwm_setpoint[1]))

        # print(pwm_setpoint)
        msg_pwm = Float64MultiArray(data=pwm_setpoint)

        # Publish pwm message
        self.pub_pwm.publish(msg_pwm)

        # Set mesasges
        if self.odom is None or self.imu is None:
            rospy.loginfo_throttle(5, "Esperando IMU (INS) y Odometry callbacks...")
            return

        # ---- Gyro desde INS (p, q, r) ----

        # ---- Gyro desde INS (p, q, r) en FRD ----
        p = getattr(self.imu.rpy_rate, "x", 0.0)
        q = getattr(self.imu.rpy_rate, "y", 0.0)
        r = getattr(self.imu.rpy_rate, "z", 0.0)
        gyro = (p, -q, -r)   # FLU -> FRD


        accel = (0.0, 0.0, 0.0)

        roll_enu  = self.imu.pose.roll
        pitch_enu = self.imu.pose.pitch
        yaw_enu   = self.imu.pose.yaw

        # ENU → NED: invierte roll y yaw
        pose_position = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        )
        pose_attitude = euler_from_quaternion([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ])

        # print("Yaw:", np.rad2deg(pose_attitude[2]))

        # ENU (odom) -> NED (vn, ve, vd)
        vxE = self.odom.twist.twist.linear.x
        vyN = self.odom.twist.twist.linear.y
        vzU = self.odom.twist.twist.linear.z
        vn, ve, vd = vyN, vxE, -vzU

        # deadband para ruido (ajusta eps si hace falta)
        def deadband(x, eps=0.05):
            return 0.0 if abs(x) < eps else x
        vn, ve, vd = map(deadband, (vn, ve, vd))

        # tasas angulares: toma tu gyro del INS (ya corregido a FRD)
        # gyro = (p, -q, -r)  # como ya tienes

        twist_linear = (vn, ve, vd, gyro[0], gyro[1], gyro[2])

        c_time = rospy.Time.now().to_sec()


        # build JSON format
        IMU_fmt = {
            "gyro" : gyro,
            "accel_body" : accel
        }
        JSON_fmt = {
            "timestamp": rospy.Time.now().to_sec(),
            "imu": {
                "gyro": [gyro[0], gyro[1], gyro[2]],          # rad/s, FRD
                "accel_body": [accel[0], accel[1], accel[2]]  # m/s^2, FRD
            },
            "position": pose_position,   # (N,E,D) en m
            "attitude": pose_attitude,   # (roll,pitch,yaw) en rad, NED
            "velocity": twist_linear     # (u,v,w,p,q,r) en FRD
        }
        rospy.loginfo_throttle(1,
            "NED pos:(%.2f,%.2f,%.2f) RPY:(%.3f,%.3f,%.3f) FRD u,v,w:(%.2f,%.2f,%.2f) p,q,r:(%.3e,%.3e,%.3e)",
            pose_position[0], pose_position[1], pose_position[2],
            pose_attitude[0], pose_attitude[1], pose_attitude[2],
            twist_linear[0], twist_linear[1], twist_linear[2],
            twist_linear[3], twist_linear[4], twist_linear[5]
        )

        JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

        # Send to AP
        self.sock_sitl.sendto(bytes(JSON_string,"ascii"), address)

        # print(self.gps.latitude)

        # gps_data = {
        #         'time_usec' : int(c_time/1e3),                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
        #         'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
        #         # 'ignore_flags' : 8,                     # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        #         # 'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
        #         # 'time_week' : 0,                        # (uint16_t) GPS week number
        #         # 'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        #         'lat' : int(self.gps.latitude*1e7),                              # (int32_t) Latitude (WGS84), in degrees * 1E7
        #         'lon' : int(self.gps.longitude*1e7),                              # (int32_t) Longitude (WGS84), in degrees * 1E7
        #         'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
        #         # 'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
        #         # 'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
        #         # 'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        #         # 've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
        #         # 'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        #         # 'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
        #         # 'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
        #         # 'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
        #         # 'satellites_visible' : 7                # (uint8_t) Number of satellites visible.
        # }

        # gps_data = json.dumps(gps_data)
        # self.gps_sock.sendto(gps_data.encode(), ("127.0.0.1", 25100))

def main():
    # Enviar JSON inicial al arrancar para que ArduSub empiece a recibir
    init_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    init_json = json.dumps({"timestamp": time.time()}) + "\n"
    init_sock.sendto(init_json.encode(), ("127.0.0.1", 9003))  # o 9002 si es blueboat
    init_sock.close()

    patch = Patch()
    rospy.spin()

if __name__ == "__main__":
    main()

