import cv2
import numpy as np

def empty(a):
    pass

cv2.namedWindow("Image")

cv2.createTrackbar("Hue Min", "Image", 0, 179, empty)
cv2.createTrackbar("Hue Max", "Image", 179, 179, empty)
cv2.createTrackbar("Sat Min", "Image", 0, 255, empty)
cv2.createTrackbar("Sat Max", "Image", 255, 255, empty)
cv2.createTrackbar("Val Min", "Image", 0, 255, empty)
cv2.createTrackbar("Val Max", "Image", 255, 255, empty)

vid = cv2.VideoCapture("vidio.webm")

while True:
    success, img = vid.read()
    cv2.imshow("Vidio", img)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue Min", "Image")
    h_max = cv2.getTrackbarPos("Hue Max", "Image")
    s_min = cv2.getTrackbarPos("Sat Min", "Image")
    s_max = cv2.getTrackbarPos("Sat Max", "Image")
    v_min = cv2.getTrackbarPos("Val Min", "Image")
    v_max = cv2.getTrackbarPos("Val Max", "Image")
    
    print(f"Hue: ({h_min}, {h_max}) Sat: ({s_min}, {s_max}) Val: ({v_min}, {v_max})")

    lower = np.array(([h_min, s_min, v_min]))
    upper = np.array(([h_max, s_max, v_max]))
    imgFilter = cv2.inRange(imgHSV, lower, upper)
    imgResult = cv2.bitwise_and(img, img, mask=imgFilter)

    cv2.imshow("Image", imgResult)
    if cv2.waitKey(100) & 0xFF ==ord('q'):
        break
