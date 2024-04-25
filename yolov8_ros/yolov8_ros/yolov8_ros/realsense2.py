#!/usr/bin/env python3

from sensor_msgs.msg import CameraInfo
import cv2
import rospy
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
import pyrealsense2 as rs

# Load the YOLOv5 model
model = YOLO('/home/kaushal/catkin_ws/src/yolov8_ros/yolov8_ros/yolov8_ros/yolov8l.pt')
#yolo = YOLO(model)
#yolo.fuse()

# Set up the RealSense D455 camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
cv_bridge = CvBridge()
#write your yolov5 depth scale here

depth_scale = 0.0010000000474974513

# Main loop
while True:
    
    # Get the latest frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Convert the frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Convert the color image to grayscale
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Convert the depth image to meters
    depth_image = depth_image * depth_scale

    

    # Detect objects using YOLOv5
    """ detection_result = model(color_image)
    print('the detected object',len(detection_result))
    result:Boxes """

    #cv_image = cv_bridge.imgmsg_to_cv2(color_image)
    results = model.predict(
        source=color_image,
        verbose=False,
        stream=False,
    )
    results: Results = results[0].cpu()
    box_data: Boxes
    # Process the results
    for box_data in results.boxes:
        box = box_data.xywh[0]
        x1, y1, w, h= box

        sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, convert_depth_to_phys_coord_using_realsense)

        def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
            _intrinsics = rs.intrinsics()
            _intrinsics.width = CameraInfo.width
            _intrinsics.height = CameraInfo.height
            _intrinsics.ppx = CameraInfo.K[2]
            _intrinsics.ppy = CameraInfo.K[5]
            _intrinsics.fx = CameraInfo.K[0]
            _intrinsics.fy = CameraInfo.K[4]
            #_intrinsics.model = CameraInfo.distortion_model
            _intrinsics.model  = rs.distortion.none
            _intrinsics.coeffs = [i for i in CameraInfo.D]
            result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
            #result[0]: right, result[1]: down, result[2]: forward
            return result[2], -result[0], -result[1]

        # Calculate the distance to the object
        #object_depth = np.median(depth_image[int(y1):int(y1+h), int(x1):int(x1+w)])
        object_depth = depth_image[int(y1), int(x1)]
        object_depth_1 = convert_depth_to_phys_coord_using_realsense(x1, y1, object_depth,  )
        label = f"{object_depth:.2f}m"

        # Draw a rectangle around the object
        cv2.rectangle(color_image, (int(x1), int(y1)), (int(x1+w), int(y1+h)), (252, 119, 30), 2)

        # Draw the bounding box
        cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

        # Print the object's class and distance
        print(f"{model.names[int(box_data.cls)]}: {object_depth:.2f}m")

    # Show the image
    cv2.imshow("Color Image", color_image)
    cv2.waitKey(1)

# Release the VideoWriter object
out.release()