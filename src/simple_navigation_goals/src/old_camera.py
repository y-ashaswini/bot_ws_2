#!/usr/bin/env python3

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import roslib
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from bob.msg import YoloResult
from std_msgs.msg import String
import time

y = YOLO('/home/alexii/robotics/bot_workspace/src/bob/models/best.pt')

class TrackerNode:
    def __init__(self):
        self.yolo_model = rospy.get_param("~yolo_model", "best.pt")
        self.result_topic = rospy.get_param("~result_topic", "yolo_result")
        self.result_image_topic = rospy.get_param("~result_image_topic", "yolo_image")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.max_det = rospy.get_param("~max_det", 300)
        self.classes = rospy.get_param("~classes", None)
        self.tracker = rospy.get_param("~tracker", "bytetrack.yaml")
        self.device = rospy.get_param("~device", None)
        self.result_conf = rospy.get_param("~result_conf", True)
        self.result_line_width = rospy.get_param("~result_line_width", None)
        self.result_font_size = rospy.get_param("~result_font_size", None)
        self.result_font = rospy.get_param("~result_font", "Arial.ttf")
        self.result_labels = rospy.get_param("~result_labels", True)
        self.result_boxes = rospy.get_param("~result_boxes", True)
        path = roslib.packages.get_pkg_dir("bob")
        self.model = YOLO(f"{path}/models/{self.yolo_model}")
        self.model.fuse()
        self.results_pub = rospy.Publisher(self.result_topic, YoloResult, queue_size=1) # YOLORESULT CUSTOM MESSAGE PUBLISHES HERE
        self.result_image_pub = rospy.Publisher(self.result_image_topic, Image, queue_size=1) ##### RESULT IMAGE PUBLISHES HERE
        self.bridge = cv_bridge.CvBridge()
        self.use_segmentation = self.yolo_model.endswith("-seg.pt")
        self.direction_pub = rospy.Publisher('direction', String, queue_size=50)
        self.last_publish_time = time.time()
        self.publish_interval = 5  # seconds

    def create_detections_array(self, results):
        detections_msg = Detection2DArray()
        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf
        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
        return detections_msg

    def create_result_image(self, results):
        plotted_image = results[0].plot(
            conf=self.result_conf,
            line_width=self.result_line_width,
            font_size=self.result_font_size,
            font=self.result_font,
            labels=self.result_labels,
            boxes=self.result_boxes,
        )
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg

    def create_segmentation_masks(self, results):
        masks_msg = []
        for result in results:
            if hasattr(result, "masks") and result.masks is not None:
                for mask_tensor in result.masks:
                    mask_numpy = (
                        np.squeeze(mask_tensor.data.to("cpu").detach().numpy()).astype(
                            np.uint8
                        )
                        * 255
                    )
                    mask_image_msg = self.bridge.cv2_to_imgmsg(
                        mask_numpy, encoding="mono8"
                    )
                    masks_msg.append(mask_image_msg)
        return masks_msg

    def capture_frames_from_webcam(self):
        cap = cv2.VideoCapture(0)  # 0 for default webcam

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                results = self.model.track(
                    source=frame,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    max_det=self.max_det,
                    classes=self.classes,
                    tracker=self.tracker,
                    device=self.device,
                    verbose=False,
                    retina_masks=True,
                )

                yolo_result_msg = YoloResult()
                yolo_result_image_msg = Image()
                yolo_result_msg.header.stamp = rospy.Time.now()

                if results is not None and len(results[0].boxes) > 0:
                    names = y.names
                    d = []
                    for r in results:
                        for c in r.boxes.cls:
                            index = int(c)
                            if 0 <= index < len(names):
                                a = names[index]
                                d.append(a)
                            else:
                                rospy.logwarn(f"Invalid index: {index} in names list.")

                    # Check if it's time to publish to "direction" topic
                    if time.time() - self.last_publish_time >= self.publish_interval:
                        self.direction_pub.publish(','.join(d))
                        self.last_publish_time = time.time()

                    yolo_result_image_msg = self.create_result_image(results)
                else:
                    d = ['no detection']
                    rospy.loginfo(d)

                    # Check if it's time to publish to "direction" topic
                    if time.time() - self.last_publish_time >= self.publish_interval:
                        self.direction_pub.publish(','.join(d)) ##### PUBLISHING DIRECTION HERE
                        self.last_publish_time = time.time()

                # yolo_result_image_msg.header.stamp = rospy.Time.now()
                yolo_result_msg.detections = self.create_detections_array(results)
                if self.use_segmentation:
                    yolo_result_msg.masks = self.create_segmentation_masks(results)
                self.results_pub.publish(yolo_result_msg) ##### PUBLISHING DETECTIONS ARRAY HERE
                self.result_image_pub.publish(yolo_result_image_msg) ##### PUBLISHING IMAGE HERE

            else:
                rospy.logerr("Error capturing frame from webcam.")
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    node.capture_frames_from_webcam()
    