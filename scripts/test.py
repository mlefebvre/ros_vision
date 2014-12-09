import numpy as np
import cv2

im = cv2.imread('/home/mathieu/Pictures/pfe/threshold2.png')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,127,255,0)


kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
thresh = cv2.erode(thresh, kernel)


contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(im, contours, -1, (0,255,0), 3)


import numpy as np

im3 = np.zeros(im.shape, np.uint8)
for contour in contours:
    area = np.abs(cv2.contourArea(contour))
    if area > 70:
        cv2.drawContours(im3, [contour], -1, (255, 255, 255), thickness=-1)

cv2.imshow("thresh", thresh)
cv2.imshow("aaa", im)
cv2.imshow("bbb", im3)

cv2.waitKey(0)


