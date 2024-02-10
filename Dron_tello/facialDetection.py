from djitellopy import tello
import cv2
import mediapipe as mp
import pygame as pg

def correct_position(amount,positive_option,negative_option):
    if amount < 0:
        negative_option(-amount)
    else:
        positive_option(amount)

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

def return_correction_factor(value,expected_value,pid):
    return pid.update(expected_value-value)

me = tello.Tello()
me.connect()
me.streamon()
cam = me.get_frame_read()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
face_detection = mp_face_detection.FaceDetection()


pid_distance = PID(0.1,0.01,0.1)
pid_rotation = PID(0.1,0.01,0.1)
pid_vertical = PID(0.1,0.01,0.1)

FINISH = False
MOVE_AMOUNT = 10
AIRBORNE = False
ACTIONS = {pg.K_w:me.move_forward,pg.K_d:me.move_right,pg.K_a:me.move_left,pg.K_s:me.move_back}
DESIRED_WIDTH = 200
DESIRED_HEIGHT = 300
CENTER_X = 0.5
CENTER_Y = 0.5

while True:
    events = pg.event.get()
    for event in events:
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_R and not AIRBORNE:
                me.takeoff()
                AIRBORNE = not AIRBORNE
            elif event.key == pg.K_r:
                me.land()
                AIRBORNE = not AIRBORNE
    keys = pg.key.get_pressed()
    for key in ACTIONS:
        if keys[key]:
            ACTIONS[key](MOVE_AMOUNT)
    if FINISH:
        break
    image = cam.frame
    cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    results = face_detection.process(image)
    if results.detections:
        
        # Selection of the face nearest to the centre
        
        selected_x_difference = abs(detection.location_data.relative_bounding_box.relative_x - CENTER_X)**2
        selected_y_difference = abs(detection.location_data.relative_bounding_box.relative_y - CENTER_Y)**2
        selected_radius_squared = selected_x_difference + selected_y_difference 
        selected_detection = results.detections[0]
        for detection in results.detections:
            mp_drawing.draw_detection(detection)
            x_difference = abs(detection.location_data.relative_bounding_box.relative_x - CENTER_X)**2
            y_difference = abs(detection.location_data.relative_bounding_box.relative_y - CENTER_Y)**2
            local_radius = x_difference + y_difference
            if local_radius < selected_radius_squared:
                local_radius = selected_radius_squared
                selected_detection = detection
        
            '''
            # Option where correction happens on every face
            if AIRBORNE:
                # Here the drone will focus on the face closest to the centre
            
            
                # Here the position correction happens
                height = detection.location_data.relative_bounding_box.height
                width = detection.location_data.relative_bounding_box.width
                relative_width = image.shape[1] * width
                relative_height = image.shape[0] * height
                # PID control
                distance_error_width = DESIRED_WIDTH - relative_width
                distance_error_height = DESIRED_HEIGHT - relative_height
                # Chosen arbitrarily, one needs to be selected as the proportions may vary
                distance_correction_factor = pid_distance.update(error_width)
                print(distance_correction_factor)
                \'''
                correct_position(distance_correction_factor,me.move_forward,me.move_back)
                \'''
            '''
        if AIRBORNE:
            # Here the distance correction happens
            height = selected_detection.location_data.relative_bounding_box.height
            width = selected_detection.location_data.relative_bounding_box.width
            x_position = selected_detection.location_data.relative_x
            y_position = selected_detection.location_data.relative_y
            relative_width = image.shape[1] * width
            relative_height = image.shape[0] * height
            # PID control
            # Width chosen arbitrarily, one needs to be selected as the proportions may vary
            distance_correction_factor = return_correction_factor(relative_width,DESIRED_WIDTH,pid_distance)
            print(distance_correction_factor)
            rotation_correction_factor = return_correction_factor(x_position,CENTER_X,pid_rotation)
            print(rotation_correction_factor)
            vertical_correction_factor = return_correction_factor(y_position,CENTER_Y,pid_vertical)
            print(vertical_correction_factor)
            '''
            correct_position(distance_correction_factor,me.move_forward,me.move_back)
            correct_position(rotation_correction_factor,me.rotate_clockwise,me.rotate_counter_clockwise)
            correct_position(vertical_correction_factor,me.move_up,me.move_down)
            '''
            # Here rotation correction happens
            
    # While testing live an issue showed up here - cv2 refuses to
    # convert the format sent from the drone
    cv2.imshow("Faces",image)
    cv2.waitKey(0)


me.land()
cv2.destroyAllWindows()