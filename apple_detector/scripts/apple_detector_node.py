#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def callback(data: Image):
    rospy.loginfo("I get image")
    
def main():    
    rospy.init_node('apple_detector', anonymous=True)

    rospy.Subscriber("/agrolabCamera/image_raw", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    main()