"""
Author: Kenzhi Wong
Project: Stamp-e with Conveyer Belt

In short, this program does the following:
1. Taking input frame from external webcam.
2. Detecting the white box coordinate, red dot coordinate, and blue dot coordinate.
3. For every time the white box passes the yellow line, publish (box, red dot, and blue dot)'s coordinate to the topics.
4. Sending the annotated frame to another ROS2 topic and show the annotated frame (note: cannot directly shown, must use cvbridge)
"""

#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import math 

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from collections import defaultdict
import numpy as np

import rclpy
from rclpy.node import Node
from my_interfaces.msg import BoxCoord, RedDotCoord, BlueDotCoord

class MyCVNode(Node):
    # ================================= INITIALIZATION =================================
    def __init__(self):
        super().__init__("mycvnode") #node name
        self.get_logger().info("The program has started.") 

        # ========================== CV - Declare Variable ==========================
        self.prev_id_box = []
        self.prev_id_red = []
        self.prev_id_blue = []
        self.track_history = defaultdict(lambda: []) # Store the track history

        self.x1_box_sent, self.y1_box_sent, self.x2_box_sent, self.y2_box_sent = 0,0,0,0
        self.red_done = False
        self.blue_done = False

        # ========================== CV - Initializing Webcam ==========================x
        self.cap = cv2.VideoCapture(0) #start webcam
        self.cap.set(3, 640) #x-length
        self.cap.set(4, 480) #y-length
        self.model = YOLO("/home/kenzhi/ma4825_ros2_ws/src/yolov8s_custom_ma4825v3_results/train/weights/best.pt") #model = YOLO("yolo-Weights/yolov8n.pt")
        self.classNames = ["blue_dot", "box", "red_dot"]
        self.br = CvBridge() #to publish and subscribe the frame

        # ========================== ROS2 - Create Publisher (to send coordinates) ==========================
        self.publish_box = self.create_publisher(BoxCoord, "box_coordinate", 10) #topic name: box_coordinate
        self.publish_red_dot = self.create_publisher(RedDotCoord, "red_dot_coordinate", 10) 
        self.publish_blue_dot = self.create_publisher(BlueDotCoord, "blue_dot_coordinate", 10)  

        # ========================== ROS2 - Create Publisher & Subscriber (to send annotated frame) ==========================
        self.publisher_img = self.create_publisher(Image, "video_frame" , 10)
        self.subscription = self.create_subscription(Image, "video_frame", self.img_callback, 10)
        
        # ========================== ROS2 - Spinning Program ==========================
        self.create_timer(0.2, self.cv_program)  #to create a timer with 0.2s betwwen each cv_program.
        self.subscription #to show the annotated frame

    # ================================= CV & ROS2 - Core Programs =================================
    def cv_program(self):

        # ====================== CV - Detect and Track the Objects ====================== 
        success, img = self.cap.read()  # Open Webcam, img = frame
        results = self.model.track(img, persist=True) # Load model

        for r in results:
            # Get the boxes, track IDs, and annotated frame
            boxes = r.boxes
            track_ids = r.boxes.id
            annotated_frame = r.plot()

            if r.boxes.id != None: #If object is detected. Note that if no object detected, id will return None, int(track_id.item() will return error.
                
                #for EVERY bounding box (either box, red dot, or blue dot) and track_id's of the bounding box:
                for box, track_id in zip(boxes, track_ids):

                    # Data Processing
                    cls = int(box.cls[0])   # class name 
                    x1, y1, x2, y2 = box.xyxy[0] # bounding box
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    # tracker
                    track = self.track_history[track_id]
                    track.append((float(x1), float(y1)))  # x, y center point
                    if len(track) > 30:  # retain 90 tracks for 90 frames
                        track.pop(0)

                    # ======================== CV - If Box Detected ======================== 
                    if self.classNames[cls] == "box":
                        if int(track_id.item()) not in self.prev_id_box: #make sure it only publishes ONCE per object

                            if x2 <= 500: # if the right edge of the box is less than 500 pixels, if it passes the yellow line
                                print(1000*"BOX ")
                                self.x1_box_sent, self.y1_box_sent, self.x2_box_sent, self.y2_box_sent = x1, y1, x2, y2

                                # ============ ROS2 - Publish Box's Coordinate ============ 
                                msg = BoxCoord()
                                msg.x1_bx, msg.y1_bx, msg.x2_bx, msg.y2_bx = self.x1_box_sent, self.y1_box_sent, self.x2_box_sent, self.y2_box_sent
                                msg.sent_done = True
                                self.publish_box.publish(msg)
                                self.prev_id_box.append(int(track_id.item()))

                    # ======================== CV - If Red Dot Detected ========================          
                    elif self.classNames[cls] == "red_dot":
                        if int(track_id.item()) not in self.prev_id_red: #make sure it only publishes ONCE per object

                            # Check if the red_dot_coord is inside the box_coord_sent
                            if (x1>self.x1_box_sent) and (y1>self.y1_box_sent) and (x2<self.x2_box_sent) and (y2<self.y2_box_sent):
                                print(1000*"RED ")
                                x1_red_sent, y1_red_sent, x2_red_sent, y2_red_sent = x1, y1, x2, y2

                                # ============ ROS2 - Publish Red Dot's Coordinate ============ 
                                msg = RedDotCoord()
                                msg.x1_r, msg.y1_r, msg.x2_r, msg.y2_r = x1_red_sent, y1_red_sent, x2_red_sent, y2_red_sent
                                msg.sent_done = True
                                self.publish_red_dot.publish(msg)
                                self.prev_id_red.append(int(track_id.item()))
                                self.red_done = True
                               
                                if self.red_done and self.blue_done: 
                                    self.x1_box_sent, self.y1_box_sent, self.x2_box_sent, self.y2_box_sent = 0,0,0,0
                                    self.red_done = False
                                    self.blue_done = False
                                         
                    # ======================== CV - If Blue Dot Detected ========================    
                    elif self.classNames[cls] == "blue_dot":
                        if int(track_id.item()) not in self.prev_id_blue: #make sure it only publishes ONCE per object

                            # Check if the blue_dot_coord is inside the box_coord_sent
                            if (x1>self.x1_box_sent) and (y1>self.y1_box_sent) and (x2<self.x2_box_sent) and (y2<self.y2_box_sent):
                                print(1000*"BLUE ")
                                x1_blue_sent, y1_blue_sent, x2_blue_sent, y2_blue_sent = x1, y1, x2, y2

                                # ============ ROS2 - Publish Blue Dot's Coordinate ============ 
                                msg = BlueDotCoord()
                                msg.x1_b, msg.y1_b, msg.x2_b, msg.y2_b = x1_blue_sent, y1_blue_sent, x2_blue_sent, y2_blue_sent
                                msg.sent_done = True
                                self.publish_blue_dot.publish(msg)
                                self.prev_id_blue.append(int(track_id.item()))
                                self.blue_done = True
                                
                                if self.red_done and self.blue_done:
                                    self.x1_box_sent, self.y1_box_sent, self.x2_box_sent, self.y2_box_sent = 0,0,0,0
                                    self.red_done = False
                                    self.blue_done = False
                    
                    # ======================== CV - Annotated Frame Drawing ========================  
                    # Put bounding box in cam
                    #cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                    # Confidence
                    confidence = math.ceil((box.conf[0]*100))/100
                    print("Confidence --->",confidence)

                    # Class name
                    cls = int(box.cls[0])
                    print("Class name -->", self.classNames[cls])

                    # Coordinate
                    coord = (x1,y1,x2,y2)

                    # Add Coordinate, Class Name, and Confidence drawing
                    org = [x1, y2]
                    org2 = [x1, y2+15]
                    org3 = [x1, y2+30]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 0.6
                    color = (30, 255, 30) # BGR
                    thickness = 2
                    
                    cv2.putText(annotated_frame, str(coord), org, font, fontScale, color, thickness) # Add text Coordinate
                    cv2.putText(annotated_frame, self.classNames[cls], org2, font, fontScale, color, thickness) # Add text Class Name
                    cv2.putText(annotated_frame, str(confidence), org3, font, fontScale, color, thickness) # Add text Confidence

                    # Draw Yellow Line
                    start_point = (500, 0)
                    end_point = (500, 480)
                    color_line = (0, 255, 255) # BGR
                    thickness_line = 4
                    cv2.line(annotated_frame, start_point, end_point, color_line, thickness_line) 

        # ====================== ROS2 - Publish the Annotated Frame to another Topic ====================== 
        self.publisher_img.publish(self.br.cv2_to_imgmsg(annotated_frame))

    # ========================== ROS2 - Annotated Frame's Subscriber Function ==========================
    def img_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("External Webcam Vision", current_frame) # show frame when this function is called
        cv2.waitKey(1)

# ========================== ROS2 - MAIN FUNCTION ==========================
def main(args=None):
    rclpy.init(args=args)
    node = MyCVNode() # Call the Node
    rclpy.spin(node)    # Loop the program
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()