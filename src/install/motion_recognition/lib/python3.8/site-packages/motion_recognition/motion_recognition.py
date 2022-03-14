import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import numpy as np
import cv2


class Counter:
    def __init__(self, frames: int):
        self.frames = frames
        self.counter = self.frames

    def decrement(self) -> None:
        self.counter -= 1

    def reset(self) -> None:
        self.counter = self.frames

    def is_finished(self) -> bool:
        return self.counter == 0


class MotionRecognizer(Node):
    def __init__(self, frames_delay: int = 10):
        super().__init__("convert_node")
        self.get_logger().info("Starting work")
        self.subscriber = self.create_subscription(Image, "/image_raw", self.listener_callback, 10)
        self.int_publisher = self.create_publisher(Int8, "/motion_flag", 10)
        self.img_publisher = self.create_publisher(Image, '/image_detection', 10)
        self.bridge = CvBridge()
        self.previous_frame = None
        self.current_frame = None
        self.kernel = np.ones((5, 5))
        self.counter = Counter(frames_delay)
        self.last_result = 0

    def _check_motion(self, img):
        if self.previous_frame is None:
            self.previous_frame = img
            self.get_logger().info('0')
            return 0, self.previous_frame

        frame_with_rectangle = self.previous_frame
        if self.counter.is_finished():
            diff_frame = cv2.absdiff(self.current_frame, self.previous_frame)
            diff_gray = cv2.cvtColor(diff_frame, cv2.COLOR_BGR2GRAY)
            diff_blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)
            diff_dilated = cv2.dilate(diff_blur, self.kernel, 1)
            thresh_frame = cv2.threshold(diff_dilated, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]

            contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                self.last_result = 1
                self.get_logger().info('1')
            else:
                self.last_result = 0
                self.get_logger().info('0')

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if cv2.contourArea(contour) > 300:
                    cv2.rectangle(frame_with_rectangle, (x, y), (x + w, y + h), (0, 255, 0), 2)

            self.previous_frame = self.current_frame
            self.counter.reset()
        else:
            self.counter.decrement()
            self.current_frame = img

        return self.last_result, frame_with_rectangle

    def listener_callback(self, msg: Image):
        img_from_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        motion_flag, detected_img = self._check_motion(img_from_cam)
        int_msg = Int8()
        int_msg.data = int(motion_flag)
        self.int_publisher.publish(int_msg)

        img_msg = self.bridge.cv2_to_imgmsg(detected_img, "bgr8")
        img_msg.header = msg.header
        self.img_publisher.publish(img_msg)


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
