import cv2
import numpy as np

# video = cv2.VideoCapture("venv/File/videonya.webm")
#
# while True:
#     success, img = video.read()
#     cv2.imshow("video",img)
#     if cv2.waitKey(40)& 0xFF == ord('q'):
#         break

#cari minimal area

gam1 = cv2.imread("venv/File/bola21.jpg")
gam2 = cv2.imread("venv/File/bola22.jpg")

gam1HSV = cv2.cvtColor(gam1,cv2.COLOR_BGR2HSV)
gam2HSV = cv2.cvtColor(gam2,cv2.COLOR_BGR2HSV)

lower_red = np.array(([160, 39, 102]))
upper_red = np.array(([179, 255, 255]))
lower_green = np.array(([52, 0, 24]))
upper_green = np.array(([124, 183, 255]))

mask_red = cv2.inRange(gam1HSV,lower_red,upper_red)
mask_green = cv2.inRange(gam2HSV,lower_green,upper_green)
imgResult_red = cv2.bitwise_and (gam1,gam1,mask=mask_red)
imgResult_green = cv2.bitwise_and(gam2,gam2,mask=mask_green)

cv2.imshow("gambar",gam1)
cv2.imshow("gambar2",gam2)
cv2.imshow("Merah",imgResult_red)
cv2.imshow("Hijau",imgResult_green)

contours,hierarchy = cv2.findContours (mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contours2,hierarchy = cv2.findContours (mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for cnt in contours:
    area = cv2.contourArea(cnt)
    print("\narea 1 = ")
    print (area)

for cnt3 in contours2:
    area2 = cv2.contourArea(cnt3)
    print("\narea2 = ")
    print(area2)

cv2.waitKey(0)
ord ('q')

# red hue = 160,179. sat = 39,255. val = 102,255.
# green hue = 52,124. sat = 0,255. val = 24,255.
#area > 200