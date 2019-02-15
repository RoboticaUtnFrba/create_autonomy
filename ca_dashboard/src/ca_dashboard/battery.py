import roslib;roslib.load_manifest('ca_dashboard')
import rospy

from rqt_robot_dashboard.widgets import BatteryDashWidget

def non_zero(value): 
   if value < 0.00001 and value > -0.00001: 
      return 0.00001 
   return value

class TurtlebotBattery(BatteryDashWidget):
    def __init__(self, name='battery'):
        icons = []
        charge_icons = []
        icons.append(['ic-battery-0.svg'])
        icons.append(['ic-battery-20.svg'])
        icons.append(['ic-battery-40.svg'])
        icons.append(['ic-battery-60-green.svg'])
        icons.append(['ic-battery-80-green.svg'])
        icons.append(['ic-battery-100-green.svg'])
        charge_icons.append(['ic-battery-charge-0.svg'])
        charge_icons.append(['ic-battery-charge-20.svg'])
        charge_icons.append(['ic-battery-charge-40.svg'])
        charge_icons.append(['ic-battery-charge-60-green.svg'])
        charge_icons.append(['ic-battery-charge-80-green.svg'])
        charge_icons.append(['ic-battery-charge-100-green.svg'])
        super(TurtlebotBattery, self).__init__(name=name, icons=icons, charge_icons=charge_icons)

        self._power_consumption = 0.0
        self._pct = 0.0
        self._cap = 2.7
        self._char_cap =2.7
        self._time_remaining = 0.0

    def set_power_state(self, msg):
        last_pct = self._pct
        last_plugged_in = self._charging
        last_time_remaining = self._time_remaining
        self._char_cap = 0.8*self._char_cap +0.2*float(msg['Charge (Ah)'])
        #make sure that battery percentage is not greater than 100%
        if self._char_cap < float(msg['Capacity (Ah)']):
          self._cap = float(msg['Capacity (Ah)'])
        else:
          self._cap = self._char_cap
    
        self._power_consumption = float(msg['Current (A)'])*float(msg['Voltage (V)'])
        #determine if we're charging or discharging
        if float(msg['Current (A)'])<0:
          tmp = (float(msg['Charge (Ah)'])/non_zero(float(msg['Current (A)'])))*60.0
        else:
          tmp = ((float(msg['Charge (Ah)'])-self._cap)/non_zero(float(msg['Current (A)'])))*60.0
    
        self._time_remaining = 0.8*self._time_remaining + 0.2*tmp
    
        self._pct = float(msg['Charge (Ah)'])/self._cap
    
        if self._pct == 1 and float(msg['Current (A)']) == 0:
            self._charging = True
        else:
            self._charging = (float(msg['Current (A)'])>0)
            
        self.update_perc(self._pct*100)
        self.update_time(self._pct*100)

