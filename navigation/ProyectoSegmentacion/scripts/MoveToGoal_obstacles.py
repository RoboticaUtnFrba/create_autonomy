#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan ## para leer los sensores
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MiNodo():

    def __init__(self):
      #############################################################################  
      ########### DECLARACION DE VARIABLES DE DE POSICION LINEAL Y ANGULAR ########
      #############################################################################
      self.x = 0.0
      self.y = 0.0
      self.yaw = 0.0
      self.roll=0.0
      self.pitch=0.0

      
      #############################################################################  
      ########### INICIALIZACION DEL NODO, SUSCRIPTORES Y PUBLISHERS ########
      #############################################################################
      
      rospy.init_node('mi_nodo')
      rospy.Subscriber("/create1/odom", Odometry, self.callback)
      self.pub = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10) ## vel publisher
      rospy.Subscriber('/create1/laser/scan', LaserScan, self.clbk_laser) ## laser suscriber
      self.vel = Twist()
      self.r = rospy.Rate(10) # 10hz

    def clbk_laser(self,msg): ## callback function for laser scan
        regions = {
            'right':  min(min(msg.ranges[:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:713]), 10),
        }

        self.take_action(regions)

    
    def take_action(self,regions):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0
     
        state_description = ''
     
        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0.0
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 2 - front'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 3 - fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 4 - fleft'
            linear_x = 0.0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 5 - front and fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 6 - front and fleft'
            linear_x = 0.0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0.0
            angular_z = -0.3
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
     
        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def normalize_angle(self, angle_diff):
        #############################################################################  
        ########### FUNCION PARA MANTENER EL ANGULO DE GIRO ENTRE - PI Y PI  ########
        #############################################################################
      
        angle_diff = np.fmod(angle_diff,2*np.pi)
        if angle_diff <0.0:
            if(abs(angle_diff) > (2*np.pi+angle_diff)):
                angle_diff = 2*np.pi+angle_diff 
        else:
            if(angle_diff > abs(angle_diff-2*np.pi) ):
                angle_diff = angle_diff - 2*np.pi
        return angle_diff

    def go_to_goal(self,goal_x,goal_y):
        ######################################################################################  
        ########### MUEVE AL ROBOT HASTA LA POSICION ESPECIFICADA POR GOAL_X Y GOAL_Y ########
        ######################################################################################
      
        while not rospy.is_shutdown():
         
         ################### CTE DE PROPORCIONALIDAD DEL CONTROLADOR DE VEL LINEAL ###
          K_linear = 0.5

          ################# CALCULO LA DISTANCIA AL OBJETIVO #################
          distance = abs(np.sqrt(((goal_x - self.x)**2) + ((goal_y - self.y)**2) ))
          
          ################## LA VEL LINEAL SE CONTROLA MUESTREANDO LA DISTANCIA PARA LLEGAR SUAVEMENTE #####
          linear_speed = distance * K_linear

          self.vel.linear.x = linear_speed
          ################ ANGULO HASTA EL OBJETIVO #############
          desired_angle_goal = np.arctan2(goal_y - self.y , goal_x - self.x)
          
          ################ CTE DE PROPORCIONALIDAD DE VEL ANGULAR #####
          K_angular = 5.0
          angle_diff = self.normalize_angle(desired_angle_goal - self.yaw)
          ################## LA VEL ANGULAR SE CONTROLA MUESTREANDO LA DISTANCIA(EN ANGULO EN RAD) PARA LLEGAR SUAVEMENTE #####
          angular_speed = angle_diff*K_angular
          self.vel.angular.z = angular_speed
          self.pub.publish(self.vel)
          print('x = ',self.x,'y = ',self.y,'angular_speed = ',angular_speed )

          if (distance<0.05):
            self.stop()
            break

          self.r.sleep()

    def stop(self):
        ######################################################################################  
        ########### FRENA AL ROBOT PARA QUE NO SIGA CON LA INERCIA ###########################
        ######################################################################################
      
        self.vel.angular.z=0.0
        self.vel.linear.x=0.0
        self.pub.publish(self.vel)

    def callback(self, data):
        ######################################################################################  
        ########### TOMO LOS VALORES DE LA ODOMETRIA EN CADA TICK  ###########################
        ######################################################################################
      
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        or_list = [orientation.x,orientation.y,orientation.z,orientation.w]
        (self.roll,self.pitch,self.yaw)=euler_from_quaternion(or_list)

if __name__ == "__main__":
    mi_nodo = MiNodo()
    mi_nodo.go_to_goal(1.0,1.0)
    mi_nodo.go_to_goal(0.0,0.0)
    mi_nodo.stop()
   