#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publish_parameters(publisher):
    fps = rospy.get_param("/stm32_node/frame_rate")
    brackets = rospy.get_param("/stm32_node/brackets")
    gain = rospy.get_param("/stereo/norlab_basler_camera_driver_node/gain")
    msg = "fps: {}/brackets: {}/gain: {}".format(fps, brackets, gain)
    publisher.publish(msg)
    return

if __name__ == '__main__':
    rospy.init_node('saving_stm32_parameters')
    pub = rospy.Publisher('stm32_parameters', String, queue_size=1)
    r = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        publish_parameters(pub)
        r.sleep()