#!/usr/bin/env python3

import socket
import json

import matplotlib.pyplot
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

class ROS():
    def __init__(self, client):
        rospy.init_node('DVL_DATA')
        self.odom = rospy.Publisher("/DVL_ODOM", Odometry, queue_size=100)
        self.doppler = rospy.Publisher(
            "/DVL_DOPPLER", Odometry, queue_size=100)

        self.dead_reckon = Odometry()
        self.doppler_data = Odometry()

        self.rate = rospy.Rate(100)  # 10hz

        self.dvl = client
        self.dvl_data = []
        self.transducer_distances = []

   
    def run(self):
        i = 0
        dist = np.zeros((4,1))
        while not rospy.is_shutdown(): # Don't need to call "run" the looping is done in other scripts
            data = json.loads(self.dvl.get().decode("utf-8"))
            # self.dvl_data = data
            # Important, data['type'] == 'velocity' printed on odd indicies, data['type'] == 'position_local' printed on even indicies
            # Individual transducer data stored within the 'velocity' type

            if i < 1000 and data['type'] == 'velocity':
                print(data['transducers'])
                for j in range(4): # data.get('transducers'):# .__sizeof__():
                    transducer = data.get('transducers').pop(0)
                    dist[j] = transducer.get('distance')
                self.transducer_distances = dist
                print(dist)
            
            i += 1

            # plot = plt.scatter(i,np.mean(dist))
            # plt.show()

            self.dead_reckon.header.stamp = rospy.Time.now()
            self.doppler_data.header.stamp = rospy.Time.now()
            if (data['type'] == 'velocity'):
                if (data['vx'] != 0):
                    self.doppler_data.twist.twist.linear.x = data['vx']
                    self.doppler_data.twist.twist.linear.y = data['vy']
                    self.doppler_data.twist.twist.linear.z = data['vz']
                    # print(data['vx'])
                    self.doppler.publish(self.doppler_data)

            elif (data['type'] == 'position_local'):
                self.dead_reckon.pose.pose.position.x = data['x']
                self.dead_reckon.pose.pose.position.y = data['y']
                self.dead_reckon.pose.pose.position.z = data['z']
                self.dead_reckon.pose.pose.orientation.x = data['roll']
                self.dead_reckon.pose.pose.orientation.y = data['pitch']
                self.dead_reckon.pose.pose.orientation.z = data['yaw']

                self.odom.publish(self.dead_reckon)

            self.rate.sleep()
        


class DVLclient():
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((TCP_IP, TCP_PORT))
            self.s.settimeout(1)
        except socket.error as err:
            sleep(1)
            self.connect()

    def get(self):
        raw_data = b''

        while True:
            rec = self.s.recv(1)
            # print(rec)
            if rec == b'\n':
                break
            raw_data += rec
        # print(raw_data)
        return (raw_data)

    def run(self):
        while True:
            self.get()


if __name__ == '__main__':

    TCP_IP = "192.168.2.95"
    TCP_PORT = 16171

    client = DVLclient(TCP_IP, TCP_PORT)
    client.connect()

    ros = ROS(client)
    ros.run()
