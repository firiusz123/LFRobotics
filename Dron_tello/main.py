from djitellopy import Tello
import pygame
from pygame.locals import *

pygame.init()

joystick = pygame.joystick.Joystick(0)
tello = Tello()

tello.connect()
#tello.takeoff()
print(tello.get_battery())

while True:
    for event in pygame.event.get(): # get the events (update the joystick)
        if event.type == QUIT: # allow to click on the X button to close the window
            pygame.quit()
            exit()
        Forward_backward = round(joystick.get_axis(0)*100)
        if Forward_backward <=5 and Forward_backward >= -5:
            Forward_backward = 0
        Left_Right = (-1)*round(joystick.get_axis(1)*100)
        if Left_Right <=5 and Left_Right >= -5:
            Left_Right = 0

        Up_down =(-1)* round(joystick.get_axis(3)*100)
        if Up_down <=5 and Up_down >= -5:
            Up_down = 0
        yaw_rotation = round(joystick.get_axis(2)*100)
        if yaw_rotation <=5 and yaw_rotation >= -5:
            yaw_rotation = 0

        if joystick.get_button(0):
            #print("A")
            tello.move_down(100)
        elif joystick.get_button(1):
            tello.land()
            #print("B")
            
        elif joystick.get_button(2):
            #print("X")
            tello.takeoff()
        elif joystick.get_button(3):
            #print("Y")
            pass
        tello.send_rc_control(left_right_velocity=Forward_backward , forward_backward_velocity= Left_Right ,up_down_velocity=Up_down , yaw_velocity=yaw_rotation)


