import cv2
import mediapipe as mp
from cvzone.HandTrackingModule import HandDetector
from math import sqrt

# Video capture set-up
cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)

width, height = 1280,720

detector = HandDetector(maxHands = 1, detectionCon = 0.8)

# To be configured
x = [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57] # Calculated distances between point 5 and 17 on the image 
y = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100] # Real distance from the screen - 
coff = np.polyfit(x, y, 2)  # Fits the above data to accurately calculate the distance y = Ax^2 + Bx + C


if not cap.isOpened():
    print("Error opening video capture")
    raise Exception("Video error")

while True:
    r, image = cap.read()
    
    hands , img = detector.findHands(image)

    # Hand detection
    # A modularized function to export
    # x,y,z = measure_hand_position(image)

    if hands:
        hand = hands[0]
        lmlist = hand['lmList']
        x, y, w, h = hands[0]['bbox']
        x1, y1, z = lmlist[5]
        x2 , y2, z = lmlist[17]
        distance = sqrt((x1-x2)**2 + (y1-y2)**2) # Picked two points on the hand and calculates their distance
        A,B,C = coff
        distance_in_centimiters = A * distance ** 2 + B * distance + C # After the fit we can calculate the approximate distance
        print(distance_in_centimiters) 
        cv2.rectangle(img, (x,y),(x + w , y + h))
    
    # Beer detection

    # Here I will use one of the libraries to train a new model - yolo or pytorch

    cv2.imshow("Hand distance", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


def detect_hands(px,py,pz,image): # First 3 arguments - previous position, the last one is the image that was read
    hands , img = detector.findHands(image)
    if hands:
        hand = hands[0]
        lmlist = hand['lmList']
        x, y, w, h = hands[0]['bbox']
        x1, y1, z = lmlist[5]
        x2 , y2, z = lmlist[17]
        distance = sqrt((x1-x2)**2 + (y1-y2)**2) # Picked two points on the hand and calculates their distance
        A,B,C = coff
        distance_in_centimiters = A * distance ** 2 + B * distance + C # After the fit we can calculate the approximate distance
        print(distance_in_centimiters) 
        cv2.rectangle(img, (x,y),(x + w , y + h))
        return (x,y,distance_in_centimiters)
    else:
        return (px,py,pz)

def detect_beer(image):
    pass