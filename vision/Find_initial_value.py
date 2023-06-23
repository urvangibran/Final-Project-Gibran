import cv2
import numpy as np


def kosong(a):
    pass

cv2.namedWindow("trackbar")
cv2.resizeWindow("trackbar", 640, 240)
cv2.createTrackbar("Hue Min", "trackbar", 0, 179, kosong)
cv2.createTrackbar("Hue Max", "trackbar", 179, 179, kosong)
cv2.createTrackbar("Sat Min", "trackbar", 0, 255, kosong)
cv2.createTrackbar("Sat Max", "trackbar", 255, 255, kosong)
cv2.createTrackbar("Val Min", "trackbar", 0, 255, kosong)
cv2.createTrackbar("Val Max", "trackbar", 255, 255, kosong)

# video = cv2.VideoCapture("venv/File/videonya.webm")
cap = cv2.imread("venv/File/bola22.jpg")

while True:
    # success, img = video.read()
    cv2.imshow("gambare", cap)
    imgHSV = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min", "trackbar")
    h_max = cv2.getTrackbarPos("Hue Max", "trackbar")
    s_min = cv2.getTrackbarPos("Sat Min", "trackbar")
    s_max = cv2.getTrackbarPos("Sat Max", "trackbar")
    v_min = cv2.getTrackbarPos("Val Min", "trackbar")
    v_max = cv2.getTrackbarPos("Val Max", "trackbar")
    print(f"Hue: ({h_min}, {h_max}) Sat: ({s_min}, {s_max}) Val: ({v_min}, {v_max})")

    lower = np.array(([h_min, s_min, v_min]))
    upper = np.array(([h_max, s_max, v_max]))
    imgFilter = cv2.inRange(imgHSV, lower, upper)
    imgResult = cv2.bitwise_and(cap, cap, mask=imgFilter)

    # cv2.imshow("Image", imgFilter)
    cv2.imshow("image", cap)
    cv2.imshow("image akhir", imgResult)
    cv2.waitKey (1)
    ord('q')
    # if cv2.waitKey(40) & 0xff == ord('q'):
    #     break

# red hue = 139,179. sat = 152,255. val = 0,255.
# green hue = 52,124. sat = 0,183. val = 24,255.