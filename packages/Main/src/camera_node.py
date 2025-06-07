#!/usr/bin/env python3

import time
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from collections import deque

# Tunable parameters
BASE_SPEED = 0.25  # Base forward speed
CURVE_SPEED = 0.2  # Reduced speed for curves
P_GAIN = 0.4       # Proportional gain
D_GAIN = 0.2       # Derivative gain - helps with curves
MAX_STEER = 0.5    # Maximum steering adjustment
SMOOTHING_STRAIGHT = 3  # Smoothing for straight roads
SMOOTHING_CURVE = 2     # Less smoothing for curves

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.actions = {
            "F": self.follow_lane,
            "R": self.turn_right,
            "S": self.go_straight,
            "L": self.turn_left
        }

        # in seconds
        self.action_times = {
            "F": 69,
            "R": 1,
            "S": 1.5,
            "L": 1.6
        }
        self.ind = 0
        self.instructions = list("FLFRFSFR")
        self.action_timer = self.action_times[self.instructions[self.ind]]

        self.p_time = time.time()

        self.p_left_motor = 0
        self.p_right_motor = 0

        self.ground_color = None


        # Setup ROS nodes
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("left_motor", Float64, queue_size=1)
        self.right_motor = rospy.Publisher("right_motor", Float64, queue_size=1)

        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)
        cv2.destroyAllWindows()

    def smooth_motor_value(self, value, history_buffer):
        """Apply smoothing to motor values"""
        history_buffer.append(value)
        return sum(history_buffer) / len(history_buffer)

    def callback(self, msg):
        if self.ind >= len(self.instructions):
            return
        
        # Process image
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)

        if self.ground_color is None:
            self.find_ground_color()

        self.vis_image = self.image.copy()

        action_end = False
        if self.instructions[self.ind] == "F":
            action_end = self.detect_red()
        else:
            action_end = self.action_timer < 0
            
        delta_time = time.time() - self.p_time
        self.p_time = time.time()

        if action_end:
            self.ind += 1
            if self.ind >= len(self.instructions):
                return
            self.action_timer = self.action_times[self.instructions[self.ind]]

        left_motor, right_motor = self.actions[self.instructions[self.ind]]()
        
        self.p_left_motor = lerp(self.p_left_motor, left_motor, 0.5)
        self.p_right_motor = lerp(self.p_right_motor, right_motor, 0.5)

        self.left_motor.publish(self.p_left_motor)
        self.right_motor.publish(self.p_right_motor)

        self.action_timer -= delta_time

        # Display visualization
        self.visualization()


    def detect_red(self):
        return False
        h, w = self.image.shape[:2]
        roi = self.image[int(h*0.7):, int(w*0.3):int(w*0.7)]

        red = np.array([50, 0, 200])

        is_red = 0
        is_not_red = 0
        for i in range(0, roi.shape[0], 10):
            for j in range(0, roi.shape[1], 10):
                if distsq(roi[i, j], red) < 400:
                    is_red += 1
                else:
                    is_not_red += 1

        return is_red > 2

    def turn_right(self):
        return 1.0, 0.7

    def turn_left(self):
        return 0.85, 1.0

    def go_straight(self):
        return 1.0, 1.0

    def follow_lane(self):
        img = self.image.copy()
        h, w = self.image.shape[:2]

        oi, oj = h//2, int(w * 0.2)
        roi = img[h//2:, int(w * 0.2):int(w * 0.8)]
        h, w = roi.shape[:2]

        total = 0
        sj = 0
        for i in range(0, h, 10):
            for j in range(0, w, 10):
                c = roi[i, j]
                is_line = True
                is_line &= distsq(c, self.ground_color) > 49*49
                is_line &= distsq(c, np.array([40, 20, 200])) > 49*49
                if is_line:
                    self.vis_image[i + oi, j + oj] = np.array([255, 255, 255])
                    sj += j
                    total += 1
        base = 0.2
        steer = (sj / total / w - 0.5) if total > 0 else 0

        return base + steer * steer, base - steer * steer
    
    def find_ground_color(self):
        h, w = self.image.shape[:2]
        roi = self.image[h-11:h-1, w//2-10:w//2+10]
        print(roi)
        s = np.array([0, 0, 0])
        for i in range(roi.shape[0]):
            for j in range(roi.shape[1]):
                s += roi[i, j]
                print(roi[i, j])
        self.ground_color = s / (roi.shape[0] * roi.shape[1])
    

    def visualization(self):
        cv2.putText(self.vis_image, f"Current Instruction: {self.instructions[self.ind]}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"instruction index: {self.ind}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"action timer: {self.action_timer}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"{self.image[0][0][0]}", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"{self.image[0][0][1]}", (60, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"{self.image[0][0][2]}", (110, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"{self.p_left_motor}", (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(self.vis_image, f"{self.p_right_motor}", (10, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow(self._window, self.vis_image)
        cv2.waitKey(1)


def distsq(a, b):
    return \
        (a[0] - b[0]) * (a[0] - b[0]) + \
        (a[1] - b[1]) * (a[1] - b[1]) + \
        (a[2] - b[2]) * (a[2] - b[2])

def lerp(a, b, t):
    return a + (b - a) * t

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()
