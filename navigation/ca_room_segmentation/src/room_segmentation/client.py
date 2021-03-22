#!/usr/bin/env python3

import actionlib
from cv_bridge import CvBridge
from cv2 import imread,flip,rotate
from geometry_msgs.msg import Point32
from ipa_building_msgs.msg import MapSegmentationGoal, MapSegmentationResult, RoomInformation, MapSegmentationAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import Image

class RoomSegmentationClient():

    def __init__(self):
      self.flag_estado = False

      # INICIALIZACION DEL NODO, SUSCRIPTORES Y PUBLISHERS
      self.rooms_dictionary = {} # diccionario para almacenar las habitaciones
      rospy.init_node('room_segmentation_client')

      # CREO UN SUSCRIPTOR A /MAP PARA OBTENER LOS PUNTOS DE ORIGEN DEL MAPA
      rospy.Subscriber("/map", OccupancyGrid, self.callback)

      # image_filename = rospkg.RosPack().get_path("ipa_room_segmentation") + "/common/files/test_maps/" + "sweet_house_5" + ".pgm"
      image_filename = rospkg.RosPack().get_path("ca_mapping") + "/maps/" + "house" + ".pgm"

      # LEO LA IMAGEN Y LA FLIPEO PARA QUE COINCIDA CON LA SEGMENTACION
      # TODO: Check the image exists
      img = imread(image_filename,0)
      img = flip(img, 0)# flipeo vertical

      arr = np.array(img)
      flat_arr = arr.ravel()
      # convert it to a matrix
      mapa = np.matrix(flat_arr)
      mapa=img

      #make non-white pixels black
      for  y  in range( 0,  len(mapa)):
        for x in range( 0 , len(mapa[0])):
          if mapa[y, x]<250: ##aca habia un cast de unsigned char
            mapa[y,x] = 0
          #else make it white
          else:
            mapa[y, x] = 255
      self.labeling = Image()
      self.cv_image = CvBridge() # aca cambie cv_image por cv_bridge

      self.cv_image.image = mapa
      self.cv_image.encoding = "mono8"
      self.labeling = self.cv_image.cv2_to_imgmsg(self.cv_image.image, self.cv_image.encoding)

      self.move_base=actionlib.SimpleActionClient("/create1/move_base", MoveBaseAction)
      rospy.logwarn("Waiting for /create1/move_base ...")
      self.move_base.wait_for_server()
      rospy.logwarn("Connected to /create1/move_base")

    def CancelGoalMoveBase(self):
      self.move_base.cancel_all_goals()

    def SendGoalSegmentation(self):
      goal2 = MapSegmentationGoal()
      goal2.input_map = self.labeling
      goal2.map_origin.position.x = 0.0  #x: -1.48144-1.481447, -6.484601
      goal2.map_origin.position.y = 0.0   #y: -6.4846
      goal2.map_resolution = 0.05
      goal2.return_format_in_meter = True
      goal2.return_format_in_pixel = False
      goal2.robot_radius = 0.4
      goal2.room_segmentation_algorithm = 3 # aca se selecciona el algoritmo 2=distance; 3=voronoi
      self.room_info =  actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server',MapSegmentationAction)
      print("Waiting for action server to start.")
      self.room_info.wait_for_server()
      #print(goal2)
      print("Action server started, sending goal.")
      self.room_info.send_goal(goal2)
      print("Goal enviado.")
      self.room_info.wait_for_result(rospy.Duration.from_sec(100.0))

      result_seg = MapSegmentationResult()
      print("resultado obtenido")
      result_seg = self.room_info.get_result()
      room_data = RoomInformation()
      room_data = result_seg.room_information_in_meter

      room_center = Point32()
      room_centers=[]
      # CARGO LOS VALORES DE LOS CENTROS A UN DICCIONARIO
      for i in range (0 ,len(room_data)):
        room_centers.append(room_data[i].room_center)
        self.load_dictionary("Cuarto {}".format(i), room_centers[i].x, room_centers[i].y)

    def load_dictionary(self,room_name,x,y):
       # CARGO EL DICIONARIO CON NOMBRES DE HABITACIONES Y SUS COORDENADAS
       self.rooms_dictionary[room_name] = (x,y)

    def callback(self,data):
      # TOMO LOS VALORES DEL PUNTO DE ORIGEN DEL MAPA PARA ACOMODAR LOS GOALS
      self.map_origin_x = data.info.origin.position.x
      self.map_origin_y = data.info.origin.position.y

    def go_to_goal(self,goal_x,goal_y):
        # MUEVE AL ROBOT HASTA LA POSICION ESPECIFICADA POR GOAL_X Y GOAL_Y
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp =rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x + self.map_origin_x
        goal.target_pose.pose.position.y= goal_y + self.map_origin_y
        goal.target_pose.pose.orientation.w=1.0

        self.move_base.send_goal(goal)

        wait = self.move_base.wait_for_result(rospy.Duration.from_sec(100.0))
        if not wait:
          rospy.logerr("Action server no disponible")
          rospy.signal_shutdown("Action server no disponible")
        else:
          res = self.move_base.get_result()

if __name__ == "__main__":
  RoomSegmentationClient()
