from rqt_robot_dashboard.widgets import ButtonDashWidget

class BreakerControl(ButtonDashWidget):
    def __init__(self, context, name, index, cb = None, icon = None):
        super(BreakerControl, self).__init__(context, name, cb, icon)

        self.clicked.connect(self.toggle)
        self._power_control = rospy.ServiceProxy('turtlebot_node/set_digital_outputs', ca_node.srv.SetDigitalOutputs)
