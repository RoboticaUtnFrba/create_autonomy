import roslib;roslib.load_manifest('ca_dashboard')
import rospy

import diagnostic_msgs
import ca_msgs.srv
import ca_msgs.msg

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, BatteryDashWidget, IconToolButton, NavViewDashWidget
from QtWidgets import QMessageBox, QAction
from python_qt_binding.QtCore import QSize

from .battery import TurtlebotBattery

import rospkg
import os.path

rp = rospkg.RosPack()

image_path = image_path = os.path.join(rp.get_path('ca_dashboard'), 'images')

class BreakerButton(IconToolButton):
    def __init__(self, name, onclick):
        self._on_icon = ['bg-green.svg', 'ic-breaker.svg']
        self._off_icon = ['bg-red.svg', 'ic-breaker.svg']
    
        icons = [self._off_icon, self._on_icon]

        super(BreakerButton, self).__init__(name, icons=icons)

        self.setFixedSize(self._icons[0].actualSize(QSize(50,30)))

        self.clicked.connect(onclick)

class CreateDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0,0,0]

        # These were moved out of get_widgets because they are sometimes not defined
        # before being used by dashboard_callback. Could be done more cleanly than this
        # though.
        self.lap_bat = BatteryDashWidget("Laptop")
        self.create_bat = TurtlebotBattery("Create")
        self.breakers = [BreakerButton('breaker0', lambda: self.toggle_breaker(0)), 
                         BreakerButton('breaker1', lambda: self.toggle_breaker(1)),
                         BreakerButton('breaker2', lambda: self.toggle_breaker(2))]
        
        # This is what gets dashboard_callback going eagerly
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
        self._power_control = rospy.ServiceProxy('turtlebot_node/set_digital_outputs', ca_msgs.srv.SetDigitalOutputs)

    def get_widgets(self):
        self.mode = MenuDashWidget('Mode')

        self.mode.add_action('Full', self.on_full_mode)
        self.mode.add_action('Passive', self.on_passive_mode)
        self.mode.add_action('Safe', self.on_safe_mode)

        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context), self.mode],
                self.breakers,
                [self.lap_bat, self.create_bat],
                [NavViewDashWidget(self.context)]]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()
  
        battery_status = {}  # Used to store TurtlebotBattery status info
        breaker_status = {}
        laptop_battery_status = {}
        op_mode = None
        for status in msg.status:
            print("Status callback %s"%status.name)
            if status.name == "/Power System/Battery":
                for value in status.values:
                    print value
                    battery_status[value.key]=value.value
                    # Create battery actually doesn't check public or check
                    # charging state. Could maybe done by looking at +-
                    # of current value
                    # Below is kobuki battery code.
                    # Best place to fix this is in create driver to match the kobuki
                    # diagnostics 
                    #elif value.key == "Charging State":
                    #    if value.value == "Trickle Charging" or value.value == "Full Charging":
                    #        self.create_bat.set_charging(True)
                    #    else:
                    #        self.create_bat.set_charging(False)
            elif status.name == "/Mode/Operating Mode":
                op_mode=status.message
            elif status.name == "/Digital IO/Digital Outputs":
                #print "got digital IO"
                for value in status.values:
                    breaker_status[value.key]=value.value
            elif status.name == "/Power System/Laptop Battery":
                for value in status.values:
                    laptop_battery_status[value.key]=value.value

        # If battery diagnostics were found, calculate percentages and stuff  
        if (laptop_battery_status):
            percentage = float(laptop_battery_status['Charge (Ah)'])/float(laptop_battery_status['Capacity (Ah)'])
            self.lap_bat.update_perc(percentage*100)
            self.lap_bat.update_time(percentage*100)
            charging_state = True if float(laptop_battery_status['Current (A)']) > 0.0 else False
            self.lap_bat.set_charging(charging_state)

        if (battery_status):
            self.create_bat.set_power_state(battery_status)

        if (breaker_status):
            self._raw_byte = int(breaker_status['Raw Byte'])
            self._update_breakers()

    def toggle_breaker(self, index):
        try:
            self.digital_outs[index] = not self.digital_outs[index]
            power_cmd = ca_msgs.srv.SetDigitalOutputsRequest(self.digital_outs[0], self.digital_outs[1], self.digital_outs[2])
            #print power_cmd
            self._power_control(power_cmd)

            return True
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Service call failed with error: %s"%(e))
            self.message.exec_()
            return False
          
        return False

    def _update_breakers(self):
        tmp = self._raw_byte
        for i in range(0,3):
            self.digital_outs[i]=tmp%2
            self.breakers[i].update_state(self.digital_outs[i])
            tmp = tmp >> 1
        
    def on_passive_mode(self):
        passive = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",ca_msgs.srv.SetTurtlebotMode )
        try:
            passive(ca_msgs.msg.TurtlebotSensorState.OI_MODE_PASSIVE)
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Failed to put the turtlebot in passive mode: service call failed with error: %s"%(e))
            self.message.exec_() 

    def on_safe_mode(self):
        safe = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",ca_msgs.srv.SetTurtlebotMode)
        try:
          safe(ca_msgs.msg.TurtlebotSensorState.OI_MODE_SAFE)
        except rospy.ServiceException, e:
          self.message = QMessageBox()
          self.message.setText("Failed to put the turtlebot in safe mode: service call failed with error: %s"%(e))
          self.message.exec_()

    def on_full_mode(self):
        full = rospy.ServiceProxy("/turtlebot_node/set_operation_mode", ca_msgs.srv.SetTurtlebotMode)
        try:
            full(ca_msgs.msg.TurtlebotSensorState.OI_MODE_FULL)
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Failed to put the turtlebot in full mode: service call failed with error: %s"%(e))
            self.message.exec_()
