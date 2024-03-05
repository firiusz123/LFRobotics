import cv2
import mediapipe as mp
from cvzone.HandTrackingModule import HandDetector
from math import sqrt

cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)

width, height = 1280,720

detector = HandDetector(maxHands = 1, detectionCon = 0.8)

if not cap.isOpened():
    print("Error opening video capture")
    raise Exception("Video error")

data = []

while True:
    r, image = cap.read()
    hands , img = detector.findHands(image)
    
    if hands:
        hand = hands[0]
        lmlist = hand['lmList']
        x1, y1, z = lmlist[5]
        x2 , y2, z = lmlist[17]
        distance = sqrt((x1-x2)**2 + (y1-y2)**2)
        print(abs(x1-x2), distance)
    cv2.imshow("Hand distance", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
