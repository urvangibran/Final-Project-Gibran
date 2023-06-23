import cv2
import numpy as np

# red hue = 139,179. sat = 152,255. val = 0,255.
# green hue = 52,124. sat = 0,183. val = 24,255.imgHSV=cv2.cvtColor(videonya, cv2.COLOR_BGR2HSV)

video = cv2.VideoCapture("venv/File/videonya.webm")

mask_red_low = np.array([139,140,10])
mask_red_high = np.array([179,255,255])

mask_green_low = np.array ([36,10,0])
mask_green_high = np.array ([120,200,255])

mask_blue_low = np.array([69,69,69])
mask_blue_high = np.array([69,69,69])

mask_yellow_low = np.array([69,69,69])
mask_yellow_high =np.array([69,69,69])


def bounding_box (img):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(imgHSV,mask_red_low,mask_red_high)
    green_mask = cv2.inRange(imgHSV, mask_green_low, mask_green_high)
    blue_mask = cv2.inRange(imgHSV, mask_blue_low, mask_blue_high)
    yellow_mask = cv2.inRange(imgHSV, mask_yellow_low, mask_yellow_high)


    red_contours,hierarchy = cv2.findContours(red_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in red_contours:
        area = cv2.contourArea(cnt)
        if area> 225 :
            # cv2.drawContours(imgHSV,cnt,-1,(0,0,255),3)
            peri = cv2.arcLength(cnt,True) # contoour length
            approx = cv2.approxPolyDP(cnt,0.02*peri,True) #contour corner points
            # corner = len(approx)
            x,y,w,h = cv2.boundingRect(approx)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(img,'RED',(x+(w//2)-10,y+(h//2-10)),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)

    green_contours, hierarchy= cv2.findContours(green_mask, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in green_contours:
        area = cv2.contourArea(cnt)
        if area > 225:
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri,True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, 'GREEN', (x+(w//2)-10,y+(h//2-10)),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,255,0),2)

    yellow_contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in yellow_contours:
        area = cv2.contourArea(cnt)
        if area > 225:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, 'YELLOW', (x + (w // 2) - 10, y + (h // 2 - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

    blue_contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in blue_contours:
        area = cv2.contourArea(cnt)
        if area > 225:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, 'BLUE', (x + (w // 2) - 10, y + (h // 2 - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

    return img

while True:
    success, img = video.read()

    bounding_boxnya = bounding_box(img)

    cv2.imshow("video",bounding_boxnya)
    if cv2.waitKey(2)& 0xFF == ord('q'):
        break