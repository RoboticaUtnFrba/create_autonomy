#!/usr/bin/env python

import rospy
import smach
import smach_ros

from ca_msgs.msg import Bumper, Cliff
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

is_left_pressed = False
is_right_pressed = False
is_virtual_wall_detected = False
cliff_msg = {'is_cliff_left': False, 'is_cliff_right': False, 'is_cliff_front_left': False,
             'is_cliff_front_right': False, 'is_cliff': False}

# define state Forward
class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['left_pressed', 'right_pressed', 'both_pressed',
                                             'virtual_wall_detected',
                                             'cliff_right', 'cliff_left'])

    def execute(self, userdata):
        rospy.loginfo('Going forward')
        global is_left_pressed, is_right_pressed, cliff_msg
        # Keep in this loop while:
        # LEFT | RIGHT | KEEP?
        #   0  |   0   |  1 --> Keep waiting
        #   0  |   1   |  0 --> right_pressed
        #   1  |   0   |  0 --> left_pressed
        #   1  |   1   |  0 --> both_pressed
        while not (is_left_pressed or is_right_pressed or is_virtual_wall_detected or cliff_msg['is_cliff']):
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
        elif is_virtual_wall_detected:
            return 'virtual_wall_detected'
        elif cliff_msg['is_cliff']:
            if cliff_msg['is_cliff_left'] or cliff_msg['is_cliff_front_left']:
                return 'cliff_left'
            if cliff_msg['is_cliff_right'] or cliff_msg['is_cliff_front_right']:
                return 'cliff_right'
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


def cliff_cb(msg):
    global cliff_msg
    cliff_msg['is_cliff_left'] = msg.is_cliff_left
    cliff_msg['is_cliff_right'] = msg.is_cliff_right
    cliff_msg['is_cliff_front_left'] = msg.is_cliff_front_left
    cliff_msg['is_cliff_front_right'] = msg.is_cliff_front_right
    cliff_msg['is_cliff'] = cliff_msg['is_cliff_left'] or cliff_msg['is_cliff_right'] or cliff_msg[
        'is_cliff_front_left'] or cliff_msg['is_cliff_front_right']


def bumper_cb(msg):
    global is_left_pressed, is_right_pressed
    is_left_pressed = msg.is_left_pressed
    is_right_pressed = msg.is_right_pressed

def virtual_wall_cb(data):
    global is_virtual_wall_detected
    is_virtual_wall_detected = data.data


def stop_cb():
    vel_pub.publish(Twist())


# main
def main():
    rospy.init_node('smach_ranking_controller')
    bumper_sub = rospy.Subscriber("bumper", Bumper, bumper_cb)
    cliff_sub = rospy.Subscriber("cliff", Cliff, cliff_cb)
    virtual_wall_sub = rospy.Subscriber("virtual_wall", Bool, virtual_wall_cb)
    global vel_pub
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.on_shutdown(stop_cb)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FORWARD', Forward(),
                               transitions={'left_pressed':'ROTATE_RIGHT_SEQ',
                                            'right_pressed':'ROTATE_LEFT_SEQ',
                                            'both_pressed': 'ROTATE_LEFT_SEQ',
                                            'virtual_wall_detected': 'ROTATE_LEFT_SEQ',
                                            'cliff_left' : 'ROTATE_RIGHT_SEQ',
                                            'cliff_right' : 'ROTATE_LEFT_SEQ'})
        smach.StateMachine.add('BACKWARD', Backward(),
                               transitions={'done':'FORWARD'})

        right_sq = smach.Sequence(
                        outcomes = ['done','aborted'],
                        connector_outcome = 'done')

        with right_sq:
            smach.Sequence.add('BACKWARD', Backward())
            smach.Sequence.add('ROTATE_RIGHT', RotateRight())

        left_sq = smach.Sequence(
                            outcomes = ['done','aborted'],
                            connector_outcome = 'done')

        with left_sq:
            smach.Sequence.add('BACKWARD', Backward())
            smach.Sequence.add('ROTATE_LEFT', RotateLeft())

        smach.StateMachine.add('ROTATE_RIGHT_SEQ', right_sq,
                               transitions={'done':'FORWARD'})
        smach.StateMachine.add('ROTATE_LEFT_SEQ', left_sq,
                               transitions={'done':'FORWARD'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_introspection', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    _ = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
