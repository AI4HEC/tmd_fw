#!/usr/bin/env python3

import rospy  # ROS Python library
from std_msgs.msg import UInt16
import numpy as np

# Create the data structure (though unused in this version)
array_data = np.empty([0, 7], dtype='object')
elephant_detect = 0

def receive_message():
    rospy.init_node('buzzer_node', anonymous=True)
   
    # Publisher for the buzzer state
    buzzer_pub = rospy.Publisher('/buzzer_state', UInt16, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        msg = UInt16()
        msg.data = 1
        buzzer_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        receive_message()
    except rospy.ROSInterruptException:
        pass
