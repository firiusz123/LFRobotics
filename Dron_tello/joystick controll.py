import pygame
from pygame.locals import *

pygame.init()

joystick = pygame.joystick.Joystick(0)

while True:
    for event in pygame.event.get(): # get the events (update the joystick)
        if event.type == QUIT: # allow to click on the X button to close the window
            pygame.quit()
            exit()

        horizontal = round(joystick.get_axis(0)*100)
        if horizontal <=5 and horizontal >= -5:
            horizontal = 0
        vertical = (-1)*round(joystick.get_axis(1)*100)
        if vertical <=5 and vertical >= -5:
            vertical = 0
        print('horizontal axis of joystick',horizontal)
        print('vertical axis of joystick',vertical)
        if joystick.get_button(0):
            print("A")
        if joystick.get_button(1):
            print("B")
        if joystick.get_button(2):
            print("X")
        if joystick.get_button(3):
            print("Y")

