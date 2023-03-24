# https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
from kalmanfilter import KalmanFilter
import cv2
from filterpy.kalman import kalman_filter
import time
# Kalman Filter
kf = KalmanFilter()

img = cv2.imread("blue_background.webp")

# ball1_positions = [(50, 100), (100, 100), (150, 100), (200, 100), (250, 100), (300, 100), (350, 100), (400, 100), (450, 100)]

ball2_positions = [(4, 300), (61, 256), (116, 214), (170, 180), (225, 148), (279, 120), (332, 97),
                   (383, 80), (434, 66), (484, 55), (535, 49), (586, 49), (634, 50),
                   (683, 58), (731, 69), (778, 82), (824,
                                                     101), (870, 124), (917, 148),
                   (962, 169), (1006, 212), (1051, 249), (1093, 290)]
ball2_positions = [(96, 127), (97, 129), (97, 132)]
for pt in ball2_positions:
    # cv2.circle(img, pt, 15, (0, 20, 220), -1)
    # predicted = kf.predict(pt[0], pt[1])

    # cv2.circle(img, predicted, 15, (20, 220, 0), 4)
    cv2.circle(img, pt, 3, (0, 20, 220), -1)
    predicted = kf.predict(pt[0], pt[1])

    cv2.circle(img, predicted, 2, (20, 220, 0), 1)
ball2_positions = [(96, 127), (97, 129), (97, 132), (96, 135), (96, 137), (95, 139), (96, 141), (97, 143), (97, 145), (97, 146), (96, 147), (96, 148), (96, 149), (95, 150), (95, 151), (94, 152), (93, 157), (91, 158), (87, 160), (84, 161), (82, 162), (81, 164), (78, 166), (76, 167), (74, 169),
                   (74, 170), (74, 171), (74, 173), (73, 175), (72, 177), (70, 180), (68, 182), (67, 184), (66, 187), (65, 191), (63, 194), (60, 196), (60, 198), (60, 201), (59, 204), (57, 206), (56, 207), (55, 209), (55, 210), (56, 211), (57, 212), (59, 215), (61, 216), (63, 218), (67, 220), (72, 222)]
for pt in ball2_positions:
    # cv2.circle(img, pt, 15, (0, 20, 220), -1)
    # predicted = kf.predict(pt[0], pt[1])

    # cv2.circle(img, predicted, 15, (20, 220, 0), 4)
    cv2.circle(img, pt, 3, (0, 20, 220), -1)
    predicted = kf.predict(pt[0], pt[1])
    # time.sleep(0.1)
    cv2.circle(img, predicted, 2, (20, 220, 0), 1)

for i in range(200):
    predicted = kf.predict(predicted[0], predicted[1])
    cv2.circle(img, predicted, 2, (0, 0, 100), 1)
    print("predicted: ", predicted)
    # cv2.circle(img, predicted, 15, (20, 220, 0), 4)

cv2.imshow("Img", img)
cv2.waitKey(0)
cv2.destroyWindow()