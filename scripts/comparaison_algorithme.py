#!/usr/bin/env python

import cv2
import numpy as np

webcam = cv2.VideoCapture(0)
frame = None

if webcam.isOpened():
    rval, frame = webcam.read()
else:
    rval = False

import time
total = 0
nb = 0

while rval:
    start = time.time()
    blur = cv2.blur(frame, (5, 5))
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    thresh1 = cv2.inRange(hsv, np.array([50, 0, 0]), np.array([110, 255, 255]))
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    erode1 = cv2.erode(thresh1, element)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    dilate1 = cv2.dilate(erode1, element)

    thresh2 = cv2.inRange(hsv, np.array([0, 153, 220]), np.array([20, 255, 255]))
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    erode2 = cv2.erode(thresh2, element)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    dilate2 = cv2.dilate(erode2, element)

    mask = cv2.bitwise_or(dilate1, dilate2)
    im2 = cv2.bitwise_and(frame, frame, mask=mask)

    rval, frame = webcam.read()
    cv2.imshow("webcam", im2)
    k = cv2.waitKey(10) & 0xFF
    if k == 27 or k == 99:
        rval = False

    nb += 1
    total += (time.time()-start)
    print (total / nb) * 1000
