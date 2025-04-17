#!/usr/bin/env python3
# import threading
# import time
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
from std_msgs.msg import Float64
# from std_msgs.msg import Bool
# from rospy.numpy_msg import numpy_msg

LEFT = 0.2
RIGHT = 0.2
GAIN = 0.2
CONST = 0.3
SLOW_RATIO = 1


class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        self.left = LEFT
        self.right = RIGHT
        self.gain = GAIN
        self.const = CONST
        self.slow_ratio = SLOW_RATIO

        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # bridge between OpenCV and ROS
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("left_motor", Float64, queue_size=1)
        self.right_motor = rospy.Publisher(
            "right_motor", Float64, queue_size=1)

        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)

        # Close the OpenCV window
        cv2.destroyAllWindows()

    def callback(self, msg):

        if self.shutting_down:
            return

        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # GOOD VALUES
        self.image = cv2.bilateralFilter(self.image, 12, 125, 155)

        luv = cv2.cvtColor(self.image, cv2.COLOR_BGR2LUV)
        yuv = cv2.cvtColor(self.image, cv2.COLOR_BGR2YUV)
        hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)

        lb_yellow = np.array([0, 0, 170])
        ub_yellow = np.array([2172, 5204, 10000])
        mask_yellow = cv2.inRange(luv, lb_yellow, ub_yellow)
        yellow_image = cv2.bitwise_and(
            self.image, self.image, mask=mask_yellow)
        yellow_image[:240, :] = 0
        kernel = np.ones((7, 7), np.uint8)
        yellow_image = cv2.dilate(yellow_image, kernel, iterations=2)

        lb_white = np.array([0, 173, 0])
        ub_white = np.array([179, 255, 255])
        mask_white = cv2.inRange(hls, lb_white, ub_white)
        white_image = cv2.bitwise_and(self.image, self.image, mask=mask_white)
        white_image[:200, :] = 0
        white_image[:, :200] = 0

        # NORMAL VALUES

        # lb_red = np.array([0, 0, 170])
        # ub_red = np.array([2172, 5204, 10000])
        # mask_red = cv2.inRange(yuv, lb_red, ub_red)
        # red_image = cv2.bitwise_and(self.image, self.image, mask=mask_red)
        # red_image[:300, :] = 0

        combined_img = cv2.bitwise_or(yellow_image, white_image)
        # combined_img = cv2.bitwise_or(combined_img, red_image)
        white_color_count = np.count_nonzero(white_image)
        yellow_color_count = np.count_nonzero(yellow_image)

        white_color_count = white_color_count / (640*480)
        yellow_color_count = yellow_color_count / (640*480)

        left_motor = (self.const + self.gain * (self.left *
                                                white_color_count)) * self.slow_ratio
        right_motor = (self.const + self.gain * (self.right *
                                                 yellow_color_count)) * self.slow_ratio

        if white_color_count < 0.1 and yellow_color_count < 0.1:
            self.left_motor.publish(-0.8)
            self.right_motor.publish(-0.8)
        elif abs(white_color_count - yellow_color_count) < 0.05:
            self.left_motor.publish(0.3)
            self.right_motor.publish(0.3)
        if white_color_count >= yellow_color_count:
            left_motor = left_motor - 0.2 if left_motor > 0.2 else left_motor
            right_motor += 0.25
            if not self.shutting_down:
                self.left_motor.publish(left_motor)
                self.right_motor.publish(right_motor)
        elif yellow_color_count > white_color_count:
            right_motor = right_motor - 0.2 if right_motor > 0.2 else right_motor
            left_motor += 0.25
            if not self.shutting_down:
                self.left_motor.publish(left_motor)
                self.right_motor.publish(right_motor)

        cv2.imshow(self._window, combined_img)
        cv2.waitKey(1)


if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')

    # keep spinning
    rospy.spin()
