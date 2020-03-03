#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 as cv

import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()


class traffic_light_fetcher_node:

    def __init__(self):
        model_location = rospy.get_param("~model_location")
        # Initializing TensorFlow

        with tf.gfile.FastGFile(model_location, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        self.sess = tf.Session()
        self.sess.graph.as_default()
        tf.import_graph_def(graph_def, name='')

        # Giving TensorFlow some time to initialize before declaring subscribers
        rospy.sleep(5.0)

        # Subscribers
        self.image_sub = rospy.Subscriber(
            "/videofile/image_raw", Image, self.callback)

        # Publishers
        self.detection_pub = rospy.Publisher(
            "/traffic_light_detected", Bool, queue_size=100)
        self.size_pub = rospy.Publisher(
            "/traffic_light_size", Vector3, queue_size=100)

        self.bridge = CvBridge()

        self.traffic_light_detected = False
        self.biggest_traffic_light_size = Vector3(x=0.0, y=0.0)

    def callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

          # Read and preprocess an image.
        rows = img.shape[0]
        cols = img.shape[1]
        inp = cv.resize(img, (640, 360))
        inp = inp[:, :, [2, 1, 0]]  # BGR2RGB

        # Run the model
        out = self.sess.run([self.sess.graph.get_tensor_by_name('num_detections:0'),
                             self.sess.graph.get_tensor_by_name(
                                 'detection_scores:0'),
                             self.sess.graph.get_tensor_by_name(
                                 'detection_boxes:0'),
                             self.sess.graph.get_tensor_by_name('detection_classes:0')],
                            feed_dict={'image_tensor:0': inp.reshape(1, inp.shape[0], inp.shape[1], 3)})

        # Processing detected bounding boxes.
        num_detections = int(out[0][0])
        for i in range(num_detections):
            classId = int(out[3][0][i])
            score = float(out[1][0][i])
            bbox = [float(v) for v in out[2][0][i]]
            if score > 0.3:
                x = bbox[1] * cols
                y = bbox[0] * rows
                right = bbox[3] * cols
                bottom = bbox[2] * rows

                # Class ID for traffic lights in COCO is 10
                if (classId == 10):
                    # Mark traffic lights in red
                    cv.rectangle(img, (int(x), int(y)), (int(right),
                                                         int(bottom)), (0, 0, 255), thickness=2)
                    # We only publish the biggest traffic light bounding box in the frame
                    if ((right - x) * (bottom - y) > self.biggest_traffic_light_size.x * self.biggest_traffic_light_size.y):
                        self.traffic_light_detected = True
                        self.biggest_traffic_light_size.x = right - x
                        self.biggest_traffic_light_size.y = bottom - y
                else:
                    # Mark other detections in green
                    cv.rectangle(img, (int(x), int(y)), (int(right),
                                                         int(bottom)), (0, 255, 0), thickness=1)
        # Publish traffic light detection status on every frame
        self.detection_pub.publish(Bool(data=self.traffic_light_detected))

        # Publish traffic light size only if it's detected
        if (self.biggest_traffic_light_size.x * self.biggest_traffic_light_size.y > 1.0):
            self.size_pub.publish(Vector3(
                x=self.biggest_traffic_light_size.x, y=self.biggest_traffic_light_size.y))

        # Reset the data
        self.traffic_light_detected = False
        self.biggest_traffic_light_size = Vector3(x=0.0, y=0.0)

        cv.imshow('Traffic light recognition', img)
        cv.waitKey(1)


def main(args):
    rospy.init_node('traffic_light_fetcher_node', anonymous=True)

    ic = traffic_light_fetcher_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()
    ic.sess.close()


if __name__ == '__main__':
    main(sys.argv)
