#!/usr/bin/env python

import rospy
import smach
import smach_ros

from ca_msgs.msg import Bumper
from geometry_msgs.msg import Twist

is_left_pressed = False
is_right_pressed = False

# define state Forward
class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['left_pressed', 'right_pressed', 'both_pressed'])

    def execute(self, userdata):
        rospy.loginfo('Going forward')
        global is_left_pressed, is_right_pressed
        while not is_left_pressed or not is_right_pressed:
            global vel_pub
            vel_msg = Twist()
            vel_msg.linear.x = 0.2
            vel_pub.publish(vel_msg)

        if is_left_pressed and is_left_pressed:
          return 'both_pressed'
        elif is_left_pressed:
          return 'left_pressed'
        elif is_right_pressed:
          return 'right_pressed'
        else:
          return 'aborted'

# define state Backward
class Backward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Going backward')
        end_time = rospy.Time.now().secs + 1.5
        while end_time > rospy.Time.now().secs:
            global vel_pub
            vel_msg = Twist()
            vel_msg.linear.x = -0.1
            vel_pub.publish(vel_msg)
        return 'done'

# define state RotateLeft
class RotateLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    
    def execute(self, userdata):
        rospy.loginfo('Rotating to the left')
        end_time = rospy.Time.now().secs + 2.
        while end_time > rospy.Time.now().secs:
            global vel_pub
            vel_msg = Twist()
            vel_msg.angular.z = 0.3
            vel_pub.publish(vel_msg)
        return 'done'

# define state RotateRight
class RotateRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    
    def execute(self, userdata):
        rospy.loginfo('Rotating to the right')
        end_time = rospy.Time.now().secs + 2.
        while end_time > rospy.Time.now().secs:
            global vel_pub
            vel_msg = Twist()
            vel_msg.angular.z = -0.3
            vel_pub.publish(vel_msg)
        return 'done'

def bumper_cb(msg):
    global is_left_pressed, is_right_pressed
    is_left_pressed = msg.is_left_pressed
    is_right_pressed = msg.is_right_pressed

def stop_cb():
    vel_pub.publish(Twist())

# main
def main():
    rospy.init_node('smach_ranking_controller')
    _ = rospy.Subscriber("bumper", Bumper, bumper_cb)
    global vel_pub
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.on_shutdown(stop_cb)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FORWARD', Forward(), 
                               transitions={'left_pressed':'ROTATE_LEFT_SEQ', 
                                            'right_pressed':'ROTATE_RIGHT_SEQ',
                                            # 'both_pressed': 'BACKWARD'})
                                            'both_pressed': 'ROTATE_LEFT_SEQ'})
        smach.StateMachine.add('BACKWARD', Backward(), 
                               transitions={'done':'FORWARD'})

        left_sq = smach.Sequence(
                        outcomes = ['done','aborted'],
                        connector_outcome = 'done')
    
        with left_sq:
            smach.Sequence.add('BACKWARD', Backward())
            smach.Sequence.add('ROTATE_RIGHT', RotateRight())
        
        right_sq = smach.Sequence(
                            outcomes = ['done','aborted'],
                            connector_outcome = 'done')
        
        with right_sq:
            smach.Sequence.add('BACKWARD', Backward())
            smach.Sequence.add('ROTATE_LEFT', RotateLeft())

        smach.StateMachine.add('ROTATE_LEFT_SEQ', right_sq, 
                               transitions={'done':'FORWARD'})
        smach.StateMachine.add('ROTATE_RIGHT_SEQ', left_sq, 
                               transitions={'done':'FORWARD'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_introspection', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    _ = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        main()
