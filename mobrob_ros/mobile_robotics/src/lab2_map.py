#!/usr/bin/env python
import math
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion as e2q
from math import pi
from std_msgs.msg import Float32MultiArray

class map():
    def __init__(self):
        self.ranges = [0,0,0,0,0,0,0,0]
        self.pose_x = 0
        self.pose_y = 0
        self.yaw = 0
        self.zakret_lasera = [-pi, -(3./4)*pi, -pi/2, -pi/4, 0, pi/4, pi/2, (3./4)*pi]
        map_width = 13.5  # m
        map_height = 13  # m
        cellsize = 20  # cells/metar
        self.dimenzija_celije = 1./cellsize #m
        self.br_celija_sirina = int(map_width*cellsize)
        self.br_celija_visina = int(map_height*cellsize)
        rospy.init_node('map_node')
        self.pub_map = rospy.Publisher('/occupancy_grid', Float32MultiArray, queue_size=1)
        self.pub_map_msg = Float32MultiArray()
        self.pub_map_msg.data = np.ones(int(map_width*cellsize*map_height*cellsize))*0.5
        self.l_k = np.zeros(int(map_width*cellsize*map_height*cellsize))
        sub_scan = rospy.Subscriber('/scan', LaserScan, self.laserScan_callback)
        sub_ground_truth_pose = rospy.Subscriber('/ground_truth_pose', Odometry, self.ground_truth_pose_callback)

        self.rov = 1.5
        self.th3db = 0.5
        self.pE = 0.15
        self.pO = 0.85
        self.deltark = 0.15

        self.cb_check_1 = False
        self.cb_check_2 = False

    def ground_truth_pose_callback(self, data):
        self.cb_check_1 = True
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.yaw = e2q([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

    def laserScan_callback(self, data):
        self.cb_check_2 = True
        self.ranges = data.ranges

    def publishing(self):
        while not self.cb_check_1 and not self.cb_check_2:
            rospy.sleep(1)
            print("Waiting for callback functions") 

        while(not rospy.is_shutdown()):
            x = self.pose_x
            y = self.pose_y
            yaw = self.yaw
            ranges = self.ranges

            for i, r in enumerate(ranges):
                if math.isnan(r):
                    r = float("inf")

                for celija_x in range(self.br_celija_sirina):
                    for celija_y in range(self.br_celija_visina):

                        x_celije = self.dimenzija_celije*celija_x+self.dimenzija_celije/2.
                        y_celije = self.dimenzija_celije*celija_y+self.dimenzija_celije/2.
                        ro = math.sqrt((x-x_celije)**2+(y-y_celije)**2)

                        if ro > self.rov:
                            continue

                        kut_celije = math.atan2(y_celije-y, x_celije-x)
                        #print('kut_celije=', kut_celije)
                        theta = kut_celije - self.zakret_lasera[i] - yaw

                        if (abs(theta) > self.th3db):
                            continue

                        #print('theta=', theta)
                        vjerojatnost_zauzetosti = self.inverzni_model_senzora(theta, ro, r)
                        #print('vjerojatnost zauzetosti=', vjerojatnost_zauzetosti)
                        indeks_celije1 = celija_x*self.br_celija_visina+self.br_celija_visina-celija_y
                        indeks_celije = self.br_celija_sirina * self.br_celija_visina -indeks_celije1 -1

                       #print('indeks celije=', indeks_celije)
                        stara_zauzetost = self.l_k[indeks_celije]
                        l_k = self.rekurzivno_racunanje_zauzetosti(stara_zauzetost, vjerojatnost_zauzetosti)
                        
                        '''
                        if abs(l_k) > 0.001:
                            print("Rob: [{0:.2f}, {1:.2f}], Celija: [{2}, {3}], Prob: {6:.4f}"
                                .format(x, y, x_celije, y_celije, ro, theta * 180 / pi, vjerojatnost_zauzetosti))
                            print("L_k: {0}, L_k-1: {1}".format(l_k, stara_zauzetost))
                            print("Ro: {0:.2f}, R: {1:.2f}, Theta: {2:.3f}".format(ro, r, theta))
                            print("")
                        '''

                        #print('l_k', l_k)
                        self.l_k[indeks_celije] = l_k
                        

            for i, l_k_ in enumerate(self.l_k):
                self.pub_map_msg.data[i] = (1./(1+math.exp(-l_k_)))

            print('publishing')
            rospy.sleep(0.01)
            self.pub_map.publish(self.pub_map_msg)

    def rekurzivno_racunanje_zauzetosti(self, stara_zauzetost, vjerojatnost_zauzetosti):
        l_0 = 0
        l_k = math.log(vjerojatnost_zauzetosti / (1 - vjerojatnost_zauzetosti)) + stara_zauzetost - l_0
        return l_k

    def inverzni_model_senzora(self, theta, ro, r):
        delta_ro = 1 - (1 + math.tanh(2 * (ro - self.rov / 100))) / 2

        if (abs(theta) > self.th3db):
            alpha = 0
        else:
            alpha = 1 - (theta / self.th3db) ** 2

        #print('Alpha: {0}, delta_ro: {1}'.format(alpha, delta_ro))

        if (ro < r - 2 * self.deltark):
            #print('Case1')
            vjerojatnost = (0.5 + (self.pE - 0.5) * alpha * delta_ro)

        elif (((r - 2 * self.deltark) <= ro) and (ro < (r - self.deltark))):
            #print('Case2')
            vjerojatnost = (0.5 + (self.pE - 0.5) * alpha * delta_ro * (1 - (2 + (ro - r) / self.deltark) ** 2))

        elif (((r - self.deltark) <= ro) and (ro < (r + self.deltark))):
            #print('Case3')
            vjerojatnost = (0.5 + (self.pO - 0.5) * alpha * delta_ro * (1 - ((ro - r) / self.deltark) ** 2))

        elif (ro > (r + self.deltark)):
            #print('Case4')
            vjerojatnost = 0.5

        return vjerojatnost


if __name__ == '__main__':
    map = map()
    map.publishing()
