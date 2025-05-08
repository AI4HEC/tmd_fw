
#!/usr/bin/env python3

import rospy  # ROS Python library
from sensor_msgs.msg import Image  # Image message type
from cv_bridge import CvBridge  # Converts ROS <-> OpenCV images
import cv2  # OpenCV library
import time

class ReolinkCamera:
    def __init__(self):
        self.camera_link = 'rtsp://admin:reolink@192.168.0.110:554/h264Preview_01_main'

        # ROS Publisher
        self.pub = rospy.Publisher('/reolink_node/image', Image, queue_size=10)
        rospy.init_node('rreolink_node_pub', anonymous=True)
        self.rate = rospy.Rate(20)  # 20 Hz

        self.br = CvBridge()

        # Initialize VideoCapture
        self.cap = None
        while self.cap is None or not self.cap.isOpened():
            try:
                self.cap = cv2.VideoCapture(self.camera_link, cv2.CAP_FFMPEG)
                if not self.cap.isOpened():
                    rospy.logwarn("Camera not opened, retrying...")
                    time.sleep(5)
            except Exception as e:
                rospy.logerr(f"Exception opening camera: {e}")
                time.sleep(5)

        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        rospy.loginfo(f"Camera connected: width={self.width}, height={self.height}")

    def rescale_frame(self, frame, percent=50):
        width = int(frame.shape[1] * percent / 100)
        height = int(frame.shape[0] * percent / 100)
        dim = (width, height)
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        frame = frame[: int(height * 0.75), :]  # crop bottom part if needed
        return frame

    def capture(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to read frame from camera.")
                continue

            resized_frame = self.rescale_frame(frame, percent=25)
            cv2.imshow('Reolink Camera View', resized_frame)
            cv2.waitKey(1)

            try:
                self.pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception as e:
                rospy.logerr(f"Failed to publish image: {e}")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        cam = ReolinkCamera()
        cam.capture()
    except rospy.ROSInterruptException:
        pass
