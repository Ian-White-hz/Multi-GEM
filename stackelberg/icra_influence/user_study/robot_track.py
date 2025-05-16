import numpy as np
import cv2
import time
from imutils.video import VideoStream
import pickle


#Start video streams
vs1 = VideoStream(src=2).start()
vs2 = VideoStream(src=4).start()
vs3 = VideoStream(src=0).start()

pose_arr  = []

counter = 0
start_time = time.time()

while(True):

    if time.time() - start_time >= 1.0:
        print("samples / second: ", counter)
        start_time = time.time()
        counter = 0

    x1 = None
    y1 = None
    x2 = None
    y2 = None
    x3 = None
    y3 = None
    frame1 = vs1.read()
    # frame1 = cv2.flip(frame1, 0)
    # frame1 = cv2.flip(frame1, 1)

    frame2 = vs2.read()
    # frame2 = cv2.flip(frame2, 0)
    # frame2 = cv2.flip(frame2, 1)

    frame3 = vs3.read()
    # frame3 = cv2.flip(frame3, 0)
    # frame3 = cv2.flip(frame3, 1)

    roi = (142, 19, 341, 461)

    clone1 = frame1.copy()
    clone2 = frame2.copy()
    clone3 = frame3.copy()

    img1 = clone1
    img2 = clone2
    img3 = clone3

    hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
    hsv3 = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)

    red_upper = np.array([170, 200, 255], np.uint8)
    red_lower = np.array([130, 50, 20], np.uint8)

    kernal = np.ones ((15, 15), "uint8")

    red1 = cv2.inRange(hsv1, red_lower, red_upper)
    red1 = cv2.morphologyEx(red1, cv2.MORPH_OPEN, kernal)
    red2 = cv2.inRange(hsv2, red_lower, red_upper)
    red2 = cv2.morphologyEx(red2, cv2.MORPH_OPEN, kernal)
    red3 = cv2.inRange(hsv3, red_lower, red_upper)
    red3 = cv2.morphologyEx(red3, cv2.MORPH_OPEN, kernal)

    # FOR CAMERA 1
    (contoursred1, hierarchy) =cv2.findContours (red1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contourred in enumerate (contoursred1):
        area = cv2.contourArea (contourred)
        if (area > 1):
            x, y, w, h = cv2.boundingRect (contourred)
            img1 = cv2.rectangle (img1, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img1,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

    if len(contoursred1) > 0:
        # Find the biggest contour
        biggest_contour = max(contoursred1, key=cv2.contourArea)

        # Find center of contour and draw filled circle
        moments = cv2.moments(biggest_contour)
        centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
        cv2.circle(img1, centre_of_contour, 2, (0, 0, 255), -1)
        # Save the center of contour so we draw line tracking it
        center_points1 = centre_of_contour
        r1 = center_points1[0]
        c1 = center_points1[1]
        y1 = r1-60
        x1 = c1-25

        x1 = 0.3+x1/35*0.1
        y1 = 0.75+y1/290*0.73
        # print(c1, r1)
        # print("CAMERA 1: target_x={}, target_y={}".format(x1,y1))

    # FOR CAMERA 2
    (contoursred2, hierarchy) =cv2.findContours (red2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contourred in enumerate (contoursred2):
        area = cv2.contourArea (contourred)
        if (area > 1):
            x, y, w, h = cv2.boundingRect (contourred)
            img2 = cv2.rectangle (img2, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img2,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

    if len(contoursred2) > 0:
        # Find the biggest contour
        biggest_contour = max(contoursred2, key=cv2.contourArea)

        # Find center of contour and draw filled circle
        moments = cv2.moments(biggest_contour)
        centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
        cv2.circle(img2, centre_of_contour, 2, (0, 0, 255), -1)
        # Save the center of contour so we draw line tracking it
        center_points1 = centre_of_contour
        r2 = center_points1[0]
        c2 = center_points1[1]
        y2 = r2-185
        x2 = c2-50

        x2 = 0.4+x2/165*0.4
        y2 = y2/295*0.75
        # print(c1, r1)
        # print("CAMERA 2: target_x={}, target_y={}".format(x2,y2))

    # FOR CAMERA 3
    (contoursred3, hierarchy) =cv2.findContours (red3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contourred in enumerate (contoursred3):
        area = cv2.contourArea (contourred)
        if (area > 1):
            x, y, w, h = cv2.boundingRect (contourred)
            img3 = cv2.rectangle (img3, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img3,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

    if len(contoursred3) > 0:
        # Find the biggest contour
        biggest_contour = max(contoursred3, key=cv2.contourArea)

        # Find center of contour and draw filled circle
        moments = cv2.moments(biggest_contour)
        centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
        cv2.circle(img3, centre_of_contour, 2, (0, 0, 255), -1)
        # Save the center of contour so we draw line tracking it
        center_points1 = centre_of_contour
        r3 = center_points1[0]
        c3 = center_points1[1]
        y3 = r3-60
        x3 = c3-50

        x3 = 0.4+x3/150*0.4
        y3 = 2.0+y3/280*0.7
        # print(c1, r1)
        # print("CAMERA 3: target_x={}, target_y={}".format(x3,y3))

    # aggregate results
    x_arr = np.array([x1,x2,x3])
    y_arr = np.array([y1,y2,y3])
    if len(contoursred1)>0 or len(contoursred2)>0 or len(contoursred3)>0:
        x_mean = []
        y_mean= []
        for idx in range(3):
            if x_arr[idx] is not None:
                x_mean.append(x_arr[idx])
                y_mean.append(y_arr[idx])
        # print("target_x={}, target_y={}".format(np.mean(x_mean), np.mean(y_mean)))
        pose_arr = [np.mean(x_mean), np.mean(y_mean)]
        pickle.dump(pose_arr, open('drone_position.pkl', 'wb'))
        counter += 1


#     cv2.imshow('Image window 1', img1)
#     cv2.imshow('Image window 2', img2)
#     cv2.imshow('Image window 3', img3)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# vs1.stop()
