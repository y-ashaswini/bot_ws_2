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
# no detection

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
        # self.results_pub = rospy.Publisher(self.result_topic, YoloResult, queue_size=1) # YOLORESULT CUSTOM MESSAGE PUBLISHES HERE
        # self.result_image_pub = rospy.Publisher(self.result_image_topic, Image, queue_size=1) ##### RESULT IMAGE PUBLISHES HERE
        self.bridge = cv_bridge.CvBridge()
        self.use_segmentation = self.yolo_model.endswith("-seg.pt")
        self.direction_pub = rospy.Publisher('direction', String, queue_size=1)
        self.last_publish_time = time.time()
        self.publish_interval = 5  # seconds
        self.depths = []
        self.dir = []

        self.detected_arrow = False
        self.center_x = 0
        self.center_y = 0  
        self.size_x = 0
        self.size_y = 0
        self.distance_appr = 0

        self.x1 = 0
        self.x2 = 0 
        self.y1 = 0
        self.y2 = 0 


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
            self.size_x = detection.bbox.size_x

            detection.bbox.size_y = float(bbox[3])
            self.size_y = detection.bbox.size_y

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


        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()


        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))


        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)

        try:
            while not rospy.is_shutdown():

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                # print(depth_image)

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))

                # Show images
                    
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.circle(images, (int(self.center_x), int(self.center_y)), 5, (0, 0, 255), -1)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

                # use depth_image to calculate distance
                # use color_image to calculate color

                # if color_image and depth_image:
                try:
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
                 


                    if results is not None and len(results[0].boxes) > 0:
                        names = y.names # y = yolo model
                        self.detected_arrow = True
                        for r in results:
                            for c in r.boxes.cls:
                                index = int(c)
                                if 0 <= index < len(names):
                                    a = names[index]
                                    self.dir.append(a)
                                else:
                                    rospy.logwarn(f"Invalid index: {index} in names list.")


                        curr_depth = depth_image[int(self.center_x)][int(self.center_y)]
                        self.distance_appr = curr_depth
                        if self.distance_appr != 0:
                            self.depths.append(self.distance_appr)
                        
                        # Check if it's time to publish to "direction" topic
                        if (time.time() - self.last_publish_time >= self.publish_interval):
                            if  (len(self.depths) != 0):
                                avg_depth = 0
                                for i in self.depths:
                                    avg_depth += i
                                avg_depth/= len(self.depths)
                                if(avg_depth <= 2000):
                                    self.direction_pub.publish(','.join(dir))
                                    self.last_publish_time = time.time()
                                    self.depths = []
                                    self.dir = []
                            else:
                                self.dir.append('no detection')

                        else:
                            # time span not over yet, append this depth into depths and these directions into dir
                            if self.distance_appr != 0:
                                self.depths.append(self.distance_appr)

                    else:
                        self.dir.append('no detection')

                        self.center_x = 0
                        self.center_y = 0  
                        self.size_x = 0
                        self.size_y = 0
                        self.distance_appr = 0

                        self.x1 = 0
                        self.x2 = 0 
                        self.y1 = 0
                        self.y2 = 0 



                except Exception as e:
                    rospy.logerr("Error: ",e)

        finally:

            # Stop streaming
            cv2.destroyAllWindows()
            pipeline.stop()


if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    node.capture_frames_from_webcam()
