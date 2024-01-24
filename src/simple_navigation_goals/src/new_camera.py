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

import pyrealsense2 as rs

y = YOLO('/home/alexii/robotics/bot_workspace/src/bob/models/best.pt')

# Left
# Right
# barrel - v1 cone barrel relabel
# nan

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
        self.device = rospy.get_param("~device", None) ##### DEVICE? NEED TO USE RGB OF REALSENSE
        self.result_conf = rospy.get_param("~result_conf", True)
        self.result_line_width = rospy.get_param("~result_line_width", None)
        self.result_font_size = rospy.get_param("~result_font_size", None)
        self.result_font = rospy.get_param("~result_font", "Arial.ttf")
        self.result_labels = rospy.get_param("~result_labels", True)
        self.result_boxes = rospy.get_param("~result_boxes", True)
        path = roslib.packages.get_pkg_dir("bob")
        self.model = YOLO(f"{path}/models/{self.yolo_model}")
        self.model.fuse()
        self.bridge = cv_bridge.CvBridge()
        self.use_segmentation = self.yolo_model.endswith("-seg.pt")

        self.direction_pub = rospy.Publisher('direction', String, queue_size=1)
        self.results_pub = rospy.Publisher(self.result_topic, YoloResult, queue_size=1) # YOLORESULT CUSTOM MESSAGE PUBLISHES HERE
        
        self.last_publish_time = time.time()
        self.curr_time = self.last_publish_time
        self.publish_interval = 5  # seconds

        self.depths = []
        self.dir = []

        self.detected_arrow = False
        self.center_x = 0
        self.center_y = 0  
        self.distance_appr = 0

        self.color_image = None
        self.depth_image = None

        self.i = 0

        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)


    def create_detections_array(self, results):
        detections_msg = Detection2DArray()

        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf

        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            self.center_x = detection.bbox.center.x

            detection.bbox.center.y = float(bbox[1])
            self.center_y = detection.bbox.center.y

            detection.bbox.size_x = float(bbox[2])
            # self.size_x = detection.bbox.size_x

            detection.bbox.size_y = float(bbox[3])
            # self.size_y = detection.bbox.size_y

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


    def rgb_callback(self, rgb_msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            if rgb_image is not None:
                self.color_image = rgb_image
        except Exception as e:
            print("rgb error: ")
            print(e)
            # rospy.logerr("Error processing RGB image: %s", str(e))



    def depth_callback(self, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            if depth_image is not None:
                self.depth_image = depth_image
            else:
                print("invalid depth img")
                # rospy.logwarn("Invalid depth image received.")

            # self.depth_image = depth_image

        except Exception as e:
            # rospy.logerr("Error processing depth image: %s", str(e))
            print("depth error: ")
            print(e)


    def capture_frames_from_webcam(self):

        if self.color_image is not None and self.depth_image is not None:
            
            try:                
                depth_image = np.asanyarray(self.depth_image)
                color_image = np.asanyarray(self.color_image)

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape


                # Print shapes for debugging


                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))

                # print("Depth Colormap Shape:", depth_colormap_dim)
                # print("Color Image Shape:", color_colormap_dim)
                
                results = self.model.track(
                    source=color_image,
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

                if results is not None and len(results[0].boxes) > 0:
                    names = y.names # y = yolo model
                    self.detected_arrow = True
                    
                    d = []
                    for r in results:
                        for c in r.boxes.cls:
                            index = int(c)
                            if 0 <= index < len(names):
                                a = names[index]
                                d.append(a)
                            else:
                                rospy.logwarn(f"Invalid index: {index} in names list.")                
                else:
                    self.detected_arrow = False
                    d = ['nan'] # no detection
                
                yolo_result_msg.detections = self.create_detections_array(results) # to save values in the center thing

                # Show images

                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.circle(images, (int(self.center_x), int(self.center_y)), 5, (0, 0, 255), -1)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)


                # depth calculations
                if(self.detected_arrow == True):
                    curr_depth = depth_image[int(self.center_y), int(self.center_x)]
                else:
                    curr_depth = -1
                # curr_depth = -1

                d.append(str(int(curr_depth))) # ['Left', [1234]] or ['', [0]] --> at the subscriber's side, if direction is ''then don't even check the depth (would be giving depth of 0,0)
                

                self.direction_pub.publish(','.join(d))
                self.results_pub.publish(yolo_result_msg) ##### PUBLISHING DETECTIONS ARRAY HERE


            except Exception as e:
                print("Error: ")
                print(e)


            self.rgb_image = None
            self.depth_image = None

        else:
            self.center_x = 0
            self.center_y = 0
        
    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.capture_frames_from_webcam()
            # if(self.i%4==0):
            #     print("capturing")
            #     self.capture_frames_from_webcam()
            #     rate.sleep()
            # self.i+=1
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("tracker_node", anonymous=True)
    node = TrackerNode()    
    node.main()
