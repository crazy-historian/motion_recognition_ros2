import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import numpy as np
import cv2


class MotionRecognizer(Node):
    def __init__(self):
        super().__init__("convert_node")
        self.get_logger().info("Starting work")
        self.subscriber = self.create_subscription(Image, "/image_raw", self.listener_callback, 10)
        self.publisher = self.create_publisher(Int8, "/motion_flag", 10)
        self.bridge = CvBridge()
        self.previous_frame = None
        self.current_frame = None
        self.kernel = np.ones((5, 5))

    def _check_motion(self, img):
        if self.previous_frame is None:
            self.previous_frame = img
            self.current_frame = img
            return 0
        self.previous_frame = self.current_frame
        self.current_frame = img

        diff_frame = cv2.absdiff(self.previous_frame, self.current_frame)
        diff_gray = cv2.cvtColor(diff_frame, cv2.COLOR_BGR2GRAY)
        diff_blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)
        diff_dilated = cv2.dilate(diff_blur, self.kernel, 1)
        thresh_frame = cv2.threshold(diff_dilated, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]

        contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            self.get_logger().info('1')
            return 1
        else:
            self.get_logger().info('0')
            return 0

    def listener_callback(self, msg: Image):
        img_from_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        motion_flag = self._check_motion(img_from_cam)
        msg = Int8()
        msg.data = int(motion_flag)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("Subscriber created")
    convert_node = MotionRecognizer()
    rclpy.spin(convert_node)
    print("Spin")
    convert_node.destroy_node()
    print("Destroyed")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
