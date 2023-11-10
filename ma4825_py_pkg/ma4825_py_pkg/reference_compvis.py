"This program is the main computer vision detection program that has not been integrated with ROS2."

from ultralytics import YOLO
import cv2
import math 

from collections import defaultdict
import numpy as np

# initializing variable
prev_id_box = []
prev_id_red = []
prev_id_blue = []
x1_box_sent, y1_box_sent, x2_box_sent, y2_box_sent = 0,0,0,0
red_done = False
blue_done = False

# start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# model
#model = YOLO("yolo-Weights/yolov8n.pt")
model = YOLO("/home/kenzhi/ma4825_ros2_ws/src/yolov8s_custom_ma4825v3_results/train/weights/best.pt")

classNames = ["blue_dot", "box", "red_dot"]

# Store the track history
track_history = defaultdict(lambda: [])

while True:
    success, img = cap.read()
    results = model.track(img, persist=True)
    print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-")

# coordinates
    for r in results:
        print("=========================================")

        # Get the boxes and track IDs
        boxes = r.boxes
        track_ids = r.boxes.id

        annotated_frame = r.plot()

        if r.boxes.id != None:
            #for box in boxes:
            for box, track_id in zip(boxes, track_ids):
                print("---------------------------------------")
                print("debugging: box:", box)
                print("track_id in track_ids", int(track_id.item()), "in", track_ids)

                # Data Processing
                cls = int(box.cls[0])   # class name

                if classNames[cls] == "box":
                    if int(track_id.item()) not in prev_id_box:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                        if x2 <= 500:
                            print(1000*"BOX ")
                            x1_box_sent = x1
                            y1_box_sent = y1
                            x2_box_sent = x2
                            y2_box_sent = y2
                            box_coordinate_sent = [x1_box_sent, y1_box_sent, x2_box_sent, y2_box_sent]
                            print("Coordinate send:", box_coordinate_sent)
                            prev_id_box.append(int(track_id.item()))

                if classNames[cls] == "red_dot":
                    if int(track_id.item()) not in prev_id_red:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                        #Check if the red_dot_coord is inside the box_coord_sent
                        if (x1>x1_box_sent) and (y1>y1_box_sent) and (x2<x2_box_sent) and (y2<y2_box_sent):
                            print(1000*"RED ")
                            x1_red_sent = x1
                            y1_red_sent = y1
                            x2_red_sent = x2
                            y2_red_sent = y2
                            red_coordinate_sent = [x1_red_sent, y1_red_sent, x2_red_sent, y2_red_sent]
                            print("Coordinate send:", red_coordinate_sent)
                            prev_id_red.append(int(track_id.item()))
                            red_done = True
                            if red_done and blue_done:
                                x1_box_sent, y1_box_sent, x2_box_sent, y2_box_sent = 0,0,0,0
                                red_done = False
                                blue_done = False                                                        

                if classNames[cls] == "blue_dot":
                    if int(track_id.item()) not in prev_id_blue:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                        #Check if the blue_dot_coord is inside the box_coord_sent
                        if (x1>x1_box_sent) and (y1>y1_box_sent) and (x2<x2_box_sent) and (y2<y2_box_sent):
                            print(1000*"BLUE ")
                            x1_blue_sent = x1
                            y1_blue_sent = y1
                            x2_blue_sent = x2
                            y2_blue_sent = y2
                            blue_coordinate_sent = [x1_blue_sent, y1_blue_sent, x2_blue_sent, y2_blue_sent]
                            print("Coordinate send:", blue_coordinate_sent)
                            prev_id_blue.append(int(track_id.item()))
                            blue_done = True
                            if red_done and blue_done:
                                x1_box_sent, y1_box_sent, x2_box_sent, y2_box_sent = 0,0,0,0
                                red_done = False
                                blue_done = False                                                        


                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                # tracker
                track = track_history[track_id]
                track.append((float(x1), float(y1)))  # x, y center point
                if len(track) > 30:  # retain 90 tracks for 90 frames
                    track.pop(0)

                # put box in cam
                #cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                # confidence
                confidence = math.ceil((box.conf[0]*100))/100
                print("Confidence --->",confidence)

                # class name
                cls = int(box.cls[0])
                print("Class name -->", classNames[cls])

                # Object drawing
                org = [x1, y2]
                org2 = [x1, y2+15]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.5
                color = (255, 0, 0)
                thickness = 2
                coord = (x1,y1,x2,y2)
                cv2.putText(annotated_frame, str(coord), org, font, fontScale, color, thickness)
                cv2.putText(annotated_frame, classNames[cls], org2, font, fontScale, color, thickness)

                start_point = (500,0)
                end_point = (500,480)
                color_line = (0, 255, 255)
                thickness_line = 4
                cv2.line(annotated_frame, start_point, end_point, color_line, thickness_line) 

    cv2.imshow('Webcam', annotated_frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()