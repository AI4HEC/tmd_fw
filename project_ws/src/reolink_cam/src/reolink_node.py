#!/usr/bin/env python3
import rospy
import subprocess
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def main():
    rospy.init_node('rtsp_republish')
    
    rtsp_source = rospy.get_param('~rtsp_source')
    output_topic = rospy.get_param('~output_topic')
    output_width = int(rospy.get_param('~output_width', 1280))
    output_height = int(rospy.get_param('~output_height', 720))
    
    pub = rospy.Publisher(output_topic, Image, queue_size=10)
    bridge = CvBridge()
    
    # GStreamer pipeline for RTSP with resize
    pipeline = (
        f"rtspsrc location={rtsp_source} latency=0 ! "
        "rtph264depay ! h264parse ! nvv4l2decoder ! "
        f"nvvidconv ! video/x-raw,width={output_width},height={output_height} ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open RTSP stream")
        return
    
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(ros_image)
            except Exception as e:
                rospy.logerr(f"Conversion error: {e}")
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
