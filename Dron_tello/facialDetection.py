from djitellopy import tello
import cv2
import mediapipe as mp
import pygame as pg

class PID:
    def __init__(self,p,i,d):
        self.kp = p
        self.ki = i
        self.kd = d
        self.previous_error = 0
        self.integral = 0
    def update(self,error,delta_time):
        # Standard PID - integrate, derive, lastly the output will determine how much the drone should move
        self.integral += error * delta_time
        derivative = (error-self.previous_error) / delta_time
        # Here we need to establish how much the drone should move away       
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.previous_error = error
        return output

me = tello.Tello()
me.connect()
me.streamon()
cam = me.get_frame_read()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
face_detection = mp_face_detection.FaceDetection()

pid = PID(0.1,0.01,0.1)

finish = False
move_amount = 10
airborne = False
actions = {pg.K_w:me.move_forward,pg.K_d:me.move_right,pg.K_a:me.move_left,pg.K_s:me.move_back}
desired_width = 200
desired_height = 300

while True:
    events = pg.event.get()
    for event in events:
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_R and not airborne:
                me.takeoff()
                airborne = not airborne
            elif event.key == pg.K_r:
                me.land()
                airborne = not airborne
    keys = pg.key.get_pressed()
    for key in actions:
        if keys[key]:
            actions[key](move_amount)
    if finish:
        break
    image = cam.frame
    cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    results = face_detection.process(image)
    if results.detections:
        for detection in results.detections:
            mp_drawing.draw_detection(detection)

            if airborne:
                # Here the position correction happens
                height = detection.location_data.relative_bounding_box.height
                width = detection.location_data.relative_bounding_box.width
                relative_width = image.shape[1] * width
                relative_height = image.shape[0] * height
                
                # PID control
                error_width = desired_width - relative_width
                error_height = desired_height - relative_height
                move_factor = pid.update(error_width)
                '''
                if move_factor < 0:
                    me.move_back(-move_factor)
                else:
                    me.move_forward(move_factor)
                '''
            print(detection)
    # While testing live an issue showed up here - cv2 refuses to
    # convert the format sent from the drone
    cv2.imshow("Faces",image)
    cv2.waitKey(0)


me.land()
cv2.destroyAllWindows()