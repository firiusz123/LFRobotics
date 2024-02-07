from djitellopy import tello
import cv2

me = tello.Tello()
me.connect()

me.streamon()

while True:
    img = me.get_frame_read().frame
    cv2.imshow('huj' , img)
    cv2.waitKey(1)