#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg

#Publishes the tag 1 pose relative to the /map frame, to "tags_gts/pose" topic
if __name__ == '__main__':
    rospy.init_node('tags_tf_2_map_pose')

    listener = tf.TransformListener()

    tag_pose_pub = rospy.Publisher('tags_gts/pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    param_name = rospy.search_param('tags_tf_2_map_rate')
    rate_param = rospy.get_param(param_name)
    rate = rospy.Rate(rate_param)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id="/map"
    seq=0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/tag_1', rospy.Time(0))
            pose.pose.position.x=trans[0]
            pose.pose.position.y=trans[1]
            pose.pose.position.z=trans[2]

            pose.header.seq=seq
            pose.header.stamp=rospy.Time.now()
            pose.pose.orientation.x=rot[0]
            pose.pose.orientation.y=rot[1]
            pose.pose.orientation.z=rot[2]
            pose.pose.orientation.w=rot[3]

            tag_pose_pub.publish(pose)
            seq+=1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(2 , "Error getting transform from /map to /tag_1")
            continue

        rate.sleep()