#!/usr/bin/env python
import csv
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion as e2q
from math import sin, cos, sqrt, atan2, isnan, floor
from numpy.linalg import multi_dot, inv
from numpy import pi
import numpy as np
import time
import rospkg
from Astarlib import show_path
from Astarlib import astar
import copy

class planiranje:


    def __init__(self):
        self.x_gt = 0
        self.y_gt = 0
        self.yaw_gt = 0
        self.x_hat = 0
        self.y_hat = 0
        self.theta_hat = 0
        self.binOccGrid = np.zeros([260, 270])
        self.checkpoints = []
        self.gt_cb_pozvan = False
        self.dim_celije = 1./20
        self.putanja = []
        self.checkpoints = []
        self.goalRadius = 0.3
        #self.goalRadius = 0.3
        self.k_ro = 3. /10
        self.k_alfa = 8./10
        self.k_beta = 1.5/10
        self.T = 0.01

        self.LINV_MAX = 0.5
        self.LINV_MIN = - 0.5
        #self.KUTNA_MAX = 0.25
        #self.KUTNA_MIN = -0.25
        self.KUTNA_MAX = 0.05
        self.KUTNA_MIN = -0.05

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mobile_robotics')
        reader = csv.reader(open(pkg_path + "/occGrid.csv"))
        for i, row in enumerate(reader):
            for j, el in enumerate(row):
                if float(el) < 0.64:
                    self.binOccGrid[259-i][j] = 0
                else:
                    self.binOccGrid[259-i][j] = 1
                    self.binOccGrid = self.addOnes(self.binOccGrid, 259-i, j, 5)
        #print(show_path(copy.deepcopy(self.binOccGrid)))
        readerCheckpoints = csv.reader(open(pkg_path + "/checkpoints.csv"))
        for row in readerCheckpoints:
            self.checkpoints.append(
                [int(floor(float(row[0])/self.dim_celije)), int(floor(float(row[1])/self.dim_celije))])
        print(self.checkpoints)
        rospy.Subscriber('/ground_truth_pose', Odometry, self.gt_cb)
        self.sub_x_k = rospy.Subscriber('/x_hat', Pose, self.x_hat_cb)
        self.pub_brzine = rospy.Publisher('/brzine', Twist, queue_size=1)
        self.pub_brzine_msg = Twist()


    def addOnes(self, map, i, j, count):
        for i_ in range(i-count, i+count):
            for j_ in range(j-count, j+count):
                try:
                    if i_ > 50 and i_ < 220 and j_ > 20 and j_ < 220:
                        map[i_][j_] = 1
                except:
                    continue

        return map

    def gt_cb(self, data):
        self.gt_cb_pozvan = True
        self.yaw_gt = e2q([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                           data.pose.pose.orientation.w])[2]
        self.x_gt = data.pose.pose.position.x
        self.y_gt = data.pose.pose.position.y

    def x_hat_cb(self, data):
        self.x_hat = data.position.x
        self.y_hat = data.position.y
        self.theta_hat = data.position.z #theta hat je kut

    def inicijalizacijaPutanje(self):
        while not self.gt_cb_pozvan and not rospy.is_shutdown():
            print('Wainting for gt message...')
            rospy.sleep(1)

        print("Pocetna pozicija: [{}, {}]".format(self.x_gt, self.y_gt))

        # Grids are reversed
        y_celije = int(floor(self.x_gt/self.dim_celije))
        x_celije = int(floor(self.y_gt/self.dim_celije))
        print("Pocetna celija: [{}, {}]".format(x_celije, y_celije))
        for goal in (self.checkpoints):
            
            # Goal grids also reversed
            x_goal = goal[1]
            y_goal = goal[0]

            #print("cilj: [{}, {}]".format(x_goal, y_goal))
            temp = astar(self.binOccGrid, (x_goal, y_goal), (x_celije, y_celije))
            self.putanja.extend(temp)
            x_celije = x_goal
            y_celije = y_goal
        show_path(copy.deepcopy(self.binOccGrid), path = self.putanja)

    def zasicenje(self, broj, min_val, max_val):
        if (broj > max_val):
            return max_val
        elif (broj < min_val):
            return min_val
        else:
            return broj 

    def pokreniRobota(self):

        control_rate = rospy.Rate(1./self.T)
        skip_put = [p for i, p in enumerate(self.putanja) if i % 1 == 0]
        skip_put.append(self.putanja[-1])

        for i, tocka in enumerate(skip_put):
            ciljna_y = tocka[0]*self.dim_celije + self.dim_celije/2.
            ciljna_x = tocka[1]*self.dim_celije + self.dim_celije/2.
            print("[{} / {}]".format(i, len(skip_put)))
            print("Start: [{}, {}], Cilj: [{}, {}]"
                .format(self.x_gt, self.y_gt, ciljna_x, ciljna_y))

            while not rospy.is_shutdown():
                x = self.x_gt
                y = self.y_gt
                ro = sqrt((x-ciljna_x)**2+(y-ciljna_y)**2)
                #print(ro)

                if(ro<self.goalRadius):
                    print("Dosegnuta ciljna tocka [{}, {}]".format(ciljna_x, ciljna_y))
                    break

                theta = self.yaw_gt
                #print("Udaljenost od iduce tocke je: {}".format(ro))
                alfa = - theta + atan2(ciljna_y-y, ciljna_x-x)

                #print("Alfa: ", alfa * 180 / pi)

                if(i+1<len(self.putanja)):
                    iduca_y = self.putanja[i+1][0] * self.dim_celije + self.dim_celije / 2.
                    iduca_x = self.putanja[i+1][1] * self.dim_celije + self.dim_celije / 2.
                    thetaG = atan2(iduca_y-ciljna_y, iduca_x-ciljna_x)
                else:
                    thetaG = 0

                beta = -theta -alfa + thetaG
                #print("Beta: ", beta * 180 / pi)
                kutnaBrzina = self.k_alfa*alfa+self.k_beta*beta
                #print("Ro: ", ro)
                linBrzina = self.k_ro*ro

                kutnaBrzina = self.zasicenje(kutnaBrzina, self.KUTNA_MIN, self.KUTNA_MAX)
                linBrzina = self.zasicenje(linBrzina, self.LINV_MIN, self.LINV_MAX)

                if (abs(abs(kutnaBrzina) - self.KUTNA_MAX) < 0.01):
                    linBrzina = 0 * linBrzina

                #print("Kutna: {0:3f}, Linearna: {0:3f}".format(kutnaBrzina, linBrzina))
                self.pub_brzine_msg.angular.z = kutnaBrzina
                self.pub_brzine_msg.linear.x = linBrzina

                self.pub_brzine.publish(self.pub_brzine_msg)
                control_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('planiranje_node')
    plan = planiranje()
    plan.inicijalizacijaPutanje()
    plan.pokreniRobota()