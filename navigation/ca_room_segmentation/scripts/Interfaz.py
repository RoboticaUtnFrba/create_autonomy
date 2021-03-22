#! /usr/bin/env python

import csv
import sys
import time

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from room_segmentation.client import RoomSegmentationClient

class RoomSegmentationThread(QThread):
  """
  Thread que controla el flag de trayectoria en una hora dada
  THREAD PARA QUE LA APP NO SE BLOQUEE CON EL MOVE BASE
  """
  def __init__(self):
    QThread.__init__(self)
    self.goal_x = []
    self.goal_y = []
    self.flag_send_goal = False
    self.flag_es_recorrido_guardado = False
    print("Arranca el thread")
    self.roomseg_client = RoomSegmentationClient()


  def run(self):
    # print(f"Flag send goal : {self.flag_send_goal}")
    # print(f"goal_x en thread : {self.goal_x}")
    # print(f"goal_y en thread : {self.goal_y}")
    while True:
      if(self.flag_send_goal == True):
        print("entra")
        for i in range(len(self.goal_x)):
          print(i)
          self.roomseg_client.CancelGoalMoveBase()
          time.sleep(1)
          self.roomseg_client.go_to_goal(self.goal_x[i],self.goal_y[i])
          print("llego a destino")
          #self.roomseg_client.CancelGoalMoveBase() # cancelo lo que este haciendo, le doy prioridad a esto
        print("reseteo flag")
        self.flag_send_goal = False
        goal_x=[] # limpio los objetivos porque solo va a realizar la trayectoria guardada
        goal_y=[]

class QtApp(QWidget):

  def __init__(self):
      super(QWidget,self).__init__()

      self.title = 'Segmentacion de mapas'
      self.left = 10
      self.top = 10
      self.width = 400
      self.height = 140
      self.Trayectoria = []
      self.grid_layout = QGridLayout()
      self.setLayout(self.grid_layout)

      self.room_names=[]
      self.combo_habitaciones = QComboBox(self)

      self.editTextTrayectoria= QLineEdit()
      self.botonGuardarTrayectoria = QPushButton('Guardar Trayectoria')
      self.editTextTime= QLineEdit()
      self.botonGuardarHora = QPushButton('Guardar Hora')

      self.label_name = QLabel("Seleccione una habitacion",self)


      self.grid_layout.addWidget(self.label_name,1,1)
      self.label_name.show()
      self.roomseg_t = RoomSegmentationThread()
      self.roomseg_t.start()

      self.roomseg_client = RoomSegmentationClient()
      self.roomseg_client.SendGoalSegmentation()
      for key in self.roomseg_client.rooms_dictionary:
        self.room_names.append(key)
      self.initUI()
      self.timer = QTimer()
      self.timer.timeout.connect(self.timer_func)
      self.timer.start(500) # timer cada 500ms que va verificando si se llego a la hora del dia guardada


  def disparar_move_base(self,saved_hour,saved_minute,saved_second):
    # FUNCION PARA CHEQUEAR LA HORA DEL DIA Y DISPARA LA TRAYECTORIA GUARDADA
    current_time = QTime.currentTime()

    current_hour = current_time.hour() - 3
    current_minute = current_time.minute()
    current_second = current_time.second()
    if current_hour == saved_hour and current_minute == saved_minute and current_second == saved_second:
      self.HoraTrayectoria()

  def timer_func(self):
    ### LEVANTA LA HORA GUARDADA DESDE UN TXT Y LA MANDA A SEND_GOAL
    Hora = [100,100,100]
    i=0
    with open('Hora.txt', 'r') as csvfile:
      spamreader = csv.reader(csvfile, delimiter=':', quotechar='|')
      for row in spamreader:
        for Time in row:
          if (int(Time)>23 or int(Time)<0) and i==0: ## por si pone mal la hora
            Time = 0
          else:
            if(int(Time) > 59 or int(Time)<0):
              Time = 0

          Hora[i]=int(Time)
          i = i+1
    i = 0
    self.disparar_move_base(Hora[0],Hora[1],Hora[2])

  def initUI(self):
      self.setWindowTitle(self.title)
      self.setGeometry(self.left, self.top, self.width, self.height)
      pos = self.frameGeometry()
      cp = QDesktopWidget().availableGeometry().center()
      pos.moveCenter(cp)
      self.move(pos.topLeft())
      # Creo un combo box con las habitaciones
      for i in self.room_names:
        self.combo_habitaciones.addItem(i)

      self.grid_layout.addWidget(self.combo_habitaciones,1,2)
      #self.combo_habitaciones.move(20,80)

      # connect button to function on_click
      self.grid_layout.addWidget(self.editTextTrayectoria,2,1)
      self.grid_layout.addWidget(self.botonGuardarTrayectoria,2,2)
      self.grid_layout.addWidget(self.editTextTime,3,1)
      self.grid_layout.addWidget(self.botonGuardarHora,3,2)

      self.botonGuardarTrayectoria.clicked.connect(self.GuardarTrayectoria)
      self.botonGuardarHora.clicked.connect(self.GuardarHora)
      ######### boton de debugueo ###########
      self.combo_habitaciones.activated[str].connect(self.SendGoalFromCB)
      self.show()

  def SendGoalFromCB(self,text):
    self.SendGoal(text)

  def HoraTrayectoria(self):
    print("HoraTrayectoria")
    self.flag_send_goal = False
    self.GetTrayectoria() ## carga las habitaciones en self.Trayectoria
    self.roomseg_client.CancelGoalMoveBase() # cancelo lo que este haciendo, le doy prioridad a esto
    self.roomseg_t.goal_x=[] # limpio los objetivos porque solo va a realizar la trayectoria guardada
    self.roomseg_t.goal_y=[]
    print("largo trayectoria")
    print(len(self.Trayectoria))
    try:
      for i in range(len(self.Trayectoria)):
        print("asdadsdas")
        room_coordinates = self.roomseg_client.rooms_dictionary[self.Trayectoria[i]]
        self.roomseg_t.goal_x.append(room_coordinates[0])
        self.roomseg_t.goal_y.append(room_coordinates[1])
        #self.roomseg_client.go_to_goal(room_coordinates[0],room_coordinates[1])
      self.flag_es_recorrido_guardado = True
      print("pase a true el flag")
      self.roomseg_t.flag_send_goal = True
      print(self.roomseg_t.flag_send_goal )
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Information)

      msg.setText("Recorrido diario comenzado ")
      msg.setInformativeText("Se esta realizando el recorrido guardado")
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()
      QApplication.processEvents()

    except:
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Warning)

      msg.setText("Error en la habitacion seleccionada!")
      msg.setInformativeText("Ingrese habitacion separadas por ,")
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()

  def GuardarTrayectoria(self):
    StringTrayectoria = self.editTextTrayectoria.text()
    print(StringTrayectoria)

    if not StringTrayectoria:
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Warning)

      msg.setText("Error en la trayectoria ingresada")
      msg.setInformativeText("Ingrese habitacion separadas por ,")
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()
    else:
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Information)

      msg.setText("Trayectoria guardada")
      msg.setInformativeText("La trayectoria fue guardada con exito")
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()
    ### AHORA GUARDAMOS LA TRAYECTORIA EN UN CSV PARA LEVANTARLA EN LA HORA ESPECIFICADA
    f = open('csv_trayectoria.csv','w')
    f.write(StringTrayectoria) #Give your csv text here.
    f.close()

  def GetTrayectoria(self):
    ### funciona ###
    self.Trayectoria = []
    with open('csv_trayectoria.csv', 'r') as csvfile:
      spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
      for row in spamreader:
        for Habitacion in row:
          self.Trayectoria.append(Habitacion)
        print("imprimo trayectoria")
        print(self.Trayectoria)

  def GuardarHora(self):
    StringHora = self.editTextTime.text()
    print(StringHora)
    cant_dot = 0
    for i in StringHora:
      if i == ':':
        cant_dot = cant_dot + 1
    if not StringHora or cant_dot != 2 :
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Warning)

      msg.setText("Error en hora ingresada")
      msg.setInformativeText("Ingrese la hora en formato HH:MM:SS ")
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()
    else:
      msg = QMessageBox()
      msg.setIcon(QMessageBox.Information)

      msg.setText("Hora guardada")
      msg.setInformativeText("Se guardo la hora para las "+str(StringHora))
      msg.setStandardButtons(QMessageBox.Ok)
      msg.exec_()
    f = open('Hora.txt','w')
    f.write(StringHora) #Give your csv text here.
    f.close()

  def SendGoal(self,text):
    self.label_name.show()

    self.label_name.setText("Navegando a la habitacion "+text+" ..." )
    #self.label_name.adjustSize()
    QApplication.processEvents()
    room_coordinates = self.roomseg_client.rooms_dictionary[text]
    self.roomseg_t.goal_x.append(room_coordinates[0])
    self.roomseg_t.goal_y.append(room_coordinates[1])

    self.roomseg_t.flag_send_goal = True
    QApplication.processEvents()

if __name__ == '__main__':
  app = QApplication(sys.argv)
  ex = QtApp()
  sys.exit(app.exec_())
