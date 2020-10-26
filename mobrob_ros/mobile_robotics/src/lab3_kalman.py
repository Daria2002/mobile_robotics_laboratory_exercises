#!/usr/bin/env python
import csv
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion as e2q
from math import sin, cos, sqrt, atan2, isnan
from numpy.linalg import multi_dot, inv
from numpy import pi
import numpy as np
import time
import rospkg
from Astarlib import show_path
from std_msgs.msg import Float64

class prosireni_kalman():
    def __init__(self):
        self.zakret_sonara = [-1.5708, -1.1220, -0.6732, -0.2244, 0.2244, 0.6732, 1.1220, 1.5708]
        self.ranges = [0, 0, 0, 0, 0, 0, 0, 0]
        self.pose_x = 0
        self.pose_y = 0
        self.yaw = 0
        self.linear_x = 0
        self.angular_z = 0
        self.x_hat = [0,0,0]
        self.P_k = np.matrix([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.05]
        ])
        self.T = 0.2
        #korijen od var_deltaTheta = sigma2
        self.var_deltaTheta = 0.02
        self.var_D = 0.05
        self.var_sonara = 0.025
        self.lista_prepreka = []
        self.cellsize = 20.
        self.dimenzija_celije = 1./20
        self.th3db = 0.025
        self.odometrija = False
        binOccGrid = np.zeros([260, 270])

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mobile_robotics')
        reader = csv.reader(open(pkg_path + "/occGrid.csv"))
        for i,row in enumerate(reader):
            for j,el in enumerate(row):
                if float(el)<0.64:
                    binOccGrid[259-i][j] = 0
                else:
                    binOccGrid[259-i][j] = 1
                    # x koordinata je j
                    self.lista_prepreka.append([j/self.cellsize+self.dimenzija_celije/2.,
                                                (260-i)/self.cellsize+self.dimenzija_celije/2.])

        show_path(binOccGrid)
        sub_odom_noise = rospy.Subscriber('/odom_noise', Odometry, self.odom_noise_callback)
        sub_odom_noise = rospy.Subscriber('/ground_truth_pose', Odometry, self.gt_cb)
        sub_scan = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        self.pub_x_k = rospy.Publisher('/x_hat', Pose, queue_size=1)
        self.pub_x_k_msg = Pose()

        self.pub_Pdet = rospy.Publisher('/P_det', Float64, queue_size=1)
        self.pub_Ptrace = rospy.Publisher('/P_trace', Float64, queue_size=1)
        self.msg_Pdet = Float64()
        self.msg_Ptrace = Float64()
       
    def odom_noise_callback(self, data):
        self.odometrija = True
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.yaw = data.pose.pose.position.z # in matlab there is calculated yaw
        #self.yaw = e2q([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.linear_x = data.twist.twist.linear.x
        self.angular_z = data.twist.twist.angular.z

    def gt_cb(self, data):
        self.yaw_gt = e2q([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
       
    def laser_scan_callback(self, data):
        self.ranges = data.ranges

    def model_robota(self, x, y, theta, D, deltaTheta, w_theta, w_D):
        '''
        :param x:
        :param y:
        :param theta:
        :param D:
        :param deltaTheta:
        :param w_theta:
        :param w_D:
        :return: returns apriori prediction x_hat_minus(k+1)
        '''
        theta_prior = theta + deltaTheta + w_theta
        x_prior = x + (D+w_D)*cos(theta_prior)
        y_prior = y + (D+w_D)*sin(theta_prior)
        x_hat_prior = np.array([x_prior, y_prior, theta_prior])
        x_hat_prior.shape=(3,1)
        return x_hat_prior

    def racunanje_P_prior(self, D, theta_hat, delta_theta, P_k):
        A = np.matrix([
            [1,0, -D*sin(theta_hat+delta_theta)],
            [0,1,D*cos(theta_hat+delta_theta)],
            [0,0,1]
        ])

        W = np.matrix([
            [-D*sin(theta_hat+delta_theta), cos(theta_hat+delta_theta)],
            [D*cos(theta_hat+delta_theta), sin(theta_hat+delta_theta)],
            [1,0]
        ])

        Q = np.matrix([
            [delta_theta**2*self.var_deltaTheta, 0],
            [0, self.var_D]
        ])
        return multi_dot([A, P_k, np.transpose(A)])+multi_dot([W, Q, np.transpose(W)])


    def model_sonara(self, x, y, x_p, y_p, v):
        '''
        :param v:
        :param x: x koordinata sonara
        :param y:
        :param x_p: x pozicija pogodjena i-tim sonarom
        :param y_p:
        :return:
        '''
        return np.sqrt((x-x_p)**2+(y-y_p)**2)+v

    def najbliza_prepreka(self, zakret_sonara, x_sonara, y_sonara, zakret_robota):
        '''
        :param x_sonara: isti kao x robota
        :param y_sonara: 
        :param zakret_sonara: 
        :return: pozicija najblize prepreke koju je udario i-ti sonar
        '''
        najmanja_udalj_prepreke = 100
        najbliza_prepreka = []
        
        for prepreka in self.lista_prepreka:
            ro = sqrt((x_sonara - prepreka[0]) ** 2 + (y_sonara - prepreka[1]) ** 2)

            if ro > 5:
                continue

            kut_celije = atan2(prepreka[1] - y_sonara, prepreka[0] - x_sonara)
            theta = kut_celije - zakret_sonara - zakret_robota

            if (abs(theta) > self.th3db):
                continue

            if ro<najmanja_udalj_prepreke:
                najmanja_udalj_prepreke = ro
                najbliza_prepreka = prepreka
           

        #if (len(najbliza_prepreka) != 0):
        #    print("Najbliza prepreka za zakret {} je [{},{}] - ro: {}"
        #        .format(zakret_sonara * 180 / pi, najbliza_prepreka[0], najbliza_prepreka[1], najmanja_udalj_prepreke))
        #else:
        #     print("Nema prepreka za zakret: {}".format(zakret_sonara*180/pi))
       
        return najbliza_prepreka

    def racunaj_H(self, P_minus, x_minus, y_minus, theta_minus, mjerenja_sonara, odstupanje):
        '''
        :param P_minus:
        :param x_minus:
        :param y_minus:
        :param mjerenja_sonara:
        :param odstupanje: max odstupanje idealnog mjerenja od stvarnog
        :return: S, H
        '''
        H = []
        inovacija = []
        v = 0 #sum
        for indeks_sonara, mjerenje in enumerate(mjerenja_sonara):

            if isnan(mjerenje):
                continue

            najbliza_prepreka = self.najbliza_prepreka(self.zakret_sonara[indeks_sonara], x_minus, y_minus, theta_minus)

            if(len(najbliza_prepreka)==0):
                continue

            idealno_mjerenje = self.model_sonara(x_minus, y_minus, najbliza_prepreka[0], najbliza_prepreka[1], v)[0]
            if(abs(idealno_mjerenje-mjerenje)>odstupanje):
                continue
            djelitelj = 1/(sqrt((x_minus-najbliza_prepreka[0])**2+(y_minus-najbliza_prepreka[1])**2))

            temp = [((x_minus-najbliza_prepreka[0])/djelitelj)[0], ((y_minus-najbliza_prepreka[1])/djelitelj)[0], 0]
            H.append(temp)
            inovacija.append(mjerenje-idealno_mjerenje)

        return  H, np.array(inovacija)

    def pokreni_Kalman(self):
        while not self.odometrija and not rospy.is_shutdown():
            print("Nema mjerenja...")
            rospy.sleep(1)

        self.x_hat = np.array([
            self.pose_x, self.pose_y, self.yaw
        ])
        self.x_hat.shape = (3, 1)

        filter_rate = rospy.Rate(1./self.T)
        while not rospy.is_shutdown():
            curr_range = self.ranges

            D = self.linear_x*self.T
            delta_theta = self.angular_z*self.T
            x_hat_minus = self.model_robota(
                self.x_hat[0],
                self.x_hat[1],
                self.x_hat[2],
                D,
                delta_theta,
                0,
                0
            )
            P_minus = self.racunanje_P_prior(D, self.x_hat[2], delta_theta, self.P_k)
            H, inovacija = self.racunaj_H(P_minus, x_hat_minus[0], x_hat_minus[1], x_hat_minus[2], curr_range, 0.3)

            if(len(H)==0):
                print("Nema korekcije")
                self.x_hat = x_hat_minus
                self.P_k = P_minus
                filter_rate.sleep()
            else:
                print("Korekcija")
                V = np.eye(len(H))
                R = np.eye(len(H)) * self.var_sonara

                #print("x_hat_minus:\n{}\n".format(x_hat_minus))
                #print("P_minus:\n{}\n".format(P_minus))
                #print("H:\n{}\n".format(H))
                #print("inovacija:\n{}\n".format(inovacija))
                
                S = multi_dot([H, P_minus, np.transpose(H)]) + multi_dot([V, R, np.transpose(V)])

                #print("matrica inovacije:\n{}\n".format(S))
                
                K = multi_dot([P_minus, np.transpose(H), inv(S)])

                print("K:\n{}\n".format(K))
                print("inovacija:\n{}\n".format(inovacija))

                self.P_k = P_minus-multi_dot([K, S, np.transpose(K)])
                self.x_hat = x_hat_minus+multi_dot([K, inovacija.reshape(-1,1)])

            print("x_hat:\n{}\n".format(self.x_hat))
            print(self.yaw_gt)
            print("P_k:\n{}\n".format(self.P_k))
            print('publishing')

            self.pub_x_k_msg.position.x = self.x_hat[0];
            self.pub_x_k_msg.position.y = self.x_hat[1];
            self.pub_x_k_msg.position.z = self.x_hat[2];
            self.pub_x_k.publish(self.pub_x_k_msg)
 
            self.msg_Ptrace.data = np.trace(self.P_k)
            self.msg_Pdet.data = np.linalg.det(self.P_k)

            self.pub_Ptrace.publish(self.msg_Ptrace)
            self.pub_Pdet.publish(self.msg_Pdet)

            print("\n\n")
            filter_rate.sleep()

if __name__=='__main__':
    rospy.init_node('kalman_node')
    kalman = prosireni_kalman()
    kalman.pokreni_Kalman()