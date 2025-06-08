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

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.actions = {
            "F": self.follow_lane,
            "R": self.turn_right,
            "S": self.go_straight,
            "L": self.turn_left,
            "I": self.initial_setup
        }

        # in seconds
        self.action_times = {
            "I":   1, # Initial
            "F":  69, # Follow lane
            "R":   1, # Turn right
            "S":   2, # Go straight
            "L": 1.7  # Turn left
        }
        self.ind = 0
        self.instructions = list("ISFLSLSLSLS")
        self.instructions = ["I"] + self.instructions
        self.action_timer = self.action_times[self.instructions[self.ind]]

        self.p_time = time.time()

        self.p_left_motor = 0
        self.p_right_motor = 0

        self.red_color = None

        # Setup ROS nodes
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.image = None
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

    def callback(self, msg):
        if self.ind >= len(self.instructions):
            return
        
        # Process image
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)

    def run(self):
        rate = rospy.Rate(30)

        while True:
            rate.sleep()

            if self.image is None:
                continue

            self.vis_image = self.image.copy()

            delta_time = time.time() - self.p_time
            self.p_time = time.time()
            self.action_timer -= delta_time

            action_end = False
            if self.instructions[self.ind] == "F":
                action_end = self.detect_red()
            else:
                action_end = self.action_timer < 0
                
            if action_end:
                self.ind += 1
                if self.ind >= len(self.instructions):
                    return
                self.action_timer = self.action_times[self.instructions[self.ind]]

            base_speed = 1.0
            left_motor, right_motor = self.actions[self.instructions[self.ind]]()
            left_motor *= base_speed
            right_motor *= base_speed
            
            self.p_left_motor = lerp(self.p_left_motor, left_motor, 1)
            self.p_right_motor = lerp(self.p_right_motor, right_motor, 1)

            self.left_motor.publish(self.p_left_motor)
            self.right_motor.publish(self.p_right_motor)

            # Display visualization
            self.visualization()



    def detect_red(self):
        return False
        h, w = self.image.shape[:2]
        oi, oj = int(h*0.7), int(w*0.3)
        h, w = h - oi, int(w * 0.7) - int(w * 0.3) - 1
        roi = self.image[oi:oi+h, oj:oj+w]

        d = 10
        offset = np.array([d, d, d])
        c_low, c_high = self.red - d, self.red + d
        mask = cv2.inRange(roi, c_low, c_high)

        mean = np.mean(mask, axis=(0, 1))

        cv2.rectangle(self.vis_image, (oj, oi), (oj + w, oi + h), (0,0,255), 2)
        seg = self.vis_image[oi:oi+h, oj:oj+w]
        self.vis_image[oi:oi+h, oj:oj+w] = cv2.bitwise_and(seg, seg, mask=mask*255)
        cv2.putText(self.vis_image, f"{mean * 100:3.2f}", (oj + 10, oi + 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 2)

        return mean > 0.25

    def initial_setup(self):
        self.red_color = self.find_ground_color()

        return 0.0, 0.0

    def turn_right(self):
        return 1.0, 0.5

    def turn_left(self):
        return 0.7, 1.0

    def go_straight(self):
        return 0,0# 1.0, 1.0

    def follow_lane(self):
        h, w = self.image.shape[:2]

        oi, oj = h//2, 0 # int(w * 0.2)
        h, w = h - h//2, w # int(w * 0.8) - int(w * 0.2) - 1

        mean = np.mean(self.image, axis=(0, 1))
        roi = cv2.GaussianBlur(np.clip((self.image[oi:oi+h, oj:oj+w] - mean) * 2 + mean, 0, 255).astype(np.uint8), (5, 5), 0)
        # roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.Canny(roi, 100, 200).astype(dtype=np.uint8)
        # mask = np.ones(mask.shape, dtype=np.uint8) * 255 - mask

        # visualization
        # self.vis_image = cv2.cvtColor(self.vis_image, cv2.COLOR_BGR2GRAY)
        seg = self.vis_image[oi:oi+h, oj:oj+w]
        self.vis_image[oi:oi+h, oj:oj+w] = cv2.bitwise_and(roi, roi, mask=np.ones(mask.shape, dtype=np.uint8) * 255 - mask)
        # visualization

        x, y = centre_of_mass(mask)
        cv2.circle(self.vis_image, (int(x) + oj, int(y) + oi), 5, 126, 5)

        steer = (1 - 2 * x / w) * 2

        base = 1
        return base + min(steer, 0), base - max(steer, 0)
    
    def find_ground_color(self):
        h, w = self.image.shape[:2]
        roi = self.image[h-11:h-1, w//2-10:w//2+10]
        return np.mean(roi, axis=(0,1))
    

    def visualization(self):
        yoff = 30

        def write(t):
            nonlocal yoff
            
            cv2.putText(self.vis_image, t, (10, yoff), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            yoff += 30

        write(f"Current Instruction: {self.instructions[self.ind]}")
        write(f"instruction index: {self.ind}")
        write(f"action timer: {self.action_timer}")
        write(f"{self.image[0][0][0]}")
        write(f"{self.image[0][0][1]}")
        write(f"{self.image[0][0][2]}")
        write(f"{self.p_left_motor}")
        write(f"{self.p_right_motor}")
        
        h, w = self.image.shape[:2]
        f = self.p_right_motor + self.p_left_motor
        x = w * (self.p_right_motor / f if f != 0 else 0.5)
        cv2.circle(self.vis_image, (int(x), h//3), 5, 126, 5)
        
        cv2.imshow(self._window, self.vis_image)
        cv2.waitKey(1)


def centre_of_mass(mask):
    """
    @brief      Finds the centre of mass of a cv2 (np uint8) binary mask. Does not use cv2.moments or contours
    @param      mask  The binary mask image
    @return     The spatial centre of mass of the mask.
    """
    
    Y, X = np.mgrid[0:mask.shape[0]:1, 0:mask.shape[1]:1]

    if np.count_nonzero(mask):
        centroid_x = int(float(np.sum(cv2.bitwise_and(X, X, mask=mask)))/np.count_nonzero(mask))
        centroid_y = int(float(np.sum(cv2.bitwise_and(Y, Y, mask=mask)))/np.count_nonzero(mask))
    else:
        centroid_x = mask.shape[1] / 2
        centroid_y = mask.shape[0] / 2

    return(centroid_x, centroid_y)

def distsq(a, b):
    return \
        (a[0] - b[0]) * (a[0] - b[0]) + \
        (a[1] - b[1]) * (a[1] - b[1]) + \
        (a[2] - b[2]) * (a[2] - b[2])

def lerp(a, b, t):
    return a + (b - a) * t

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    node.run()
    rospy.spin()
