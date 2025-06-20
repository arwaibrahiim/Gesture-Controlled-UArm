import mediapipe as mp
import numpy as np
import cv2 
import pyfirmata
import time
import math
import sim

# Smooth servo movement function
def low_pass_filter(old_filtered_value, new_value, alpha = 0.1):
    if old_filtered_value == None:
        old_filtered_value = new_value
    filtered_value = alpha * new_value + (1-alpha) * old_filtered_value
    return filtered_value

# Home position function
def home_position():
    servos[0].write(90)
    time.sleep(0.1)
    servos[1].write(40)
    time.sleep(0.1)
    servos[2].write(90)
    time.sleep(0.1)
    servos[3].write(90)
    time.sleep(0.1)
    servos[4].write(90)
    time.sleep(0.1)
    
# Distance calculation function 
def calculate_distance(x1,x2,y1,y2):
    return (math.sqrt((x1 - x2)**2 + (y1 - y2)**2))

# Gripper angle mapping function
def map_angle(distance, min_dist, max_dist, min_angle, max_angle):
    distance = max(min_dist, min(distance, max_dist))
    angle = min_angle + (distance - min_dist) * (max_angle - min_angle) / (max_dist - min_dist)
    return angle

# Coordinate mapping function
def map_value(value, in_min, in_max, out_min, out_max):
    return max(out_min, min(out_max, (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

# Buzzer trigger function
def buzzer_feedback():
    buzzer.write(1)
    time.sleep(0.5)
    buzzer.write(0)
    

    
# Board initialization
board = pyfirmata.Arduino('COM5')

# Start iterator
it = pyfirmata.util.Iterator(board)
it.start()

# Servo pins initialization
servos = [board.get_pin('d:11:s'), 
          board.get_pin('d:10:s'), 
          board.get_pin('d:5:s'), 
          board.get_pin('d:6:s'), 
          board.get_pin('d:9:s')]
buzzer = board.get_pin('d:3:o')

# Code start
home_position()
time.sleep(0.1)

#------------------- CoppeliaSim API Starting ----------------------#

sim.simxFinish(-1)  # Close all opened connections
client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
timestep = 0.01
print("Connected to CoppeliaSim")

# Set up handles
_, object_handle = sim.simxGetObjectHandle(client_id, 'Target', sim.simx_opmode_blocking)
_, gripper_handle = sim.simxGetObjectHandle(client_id, 'uarmGripper', sim.simx_opmode_blocking)
    
joint_handles = []
for i in range(1, 4): 
    _, joint_handle = sim.simxGetObjectHandle(client_id, f'M_Joint{i}', sim.simx_opmode_blocking)
    joint_handles.append(joint_handle)

# Initialize streaming
_, initial_pos = sim.simxGetObjectPosition(client_id, object_handle, -1, sim.simx_opmode_streaming)
sim.simxGetObjectPosition(client_id, gripper_handle, -1, sim.simx_opmode_streaming)

for handle in joint_handles:
    sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_streaming)
    
     
#------------------- Main ----------------------#

# Needed initialization
y_coppelia = 0
z_coppelia = 0
x_coppelia = 0
gripper_angle = 90
movement_enabled = False

mp_drawing = mp.solutions.drawing_utils
mp_hand = mp.solutions.hands


prev_x, prev_y, prev_z = None, None, None

# Camera setup and main loop
with mp_hand.Hands(min_detection_confidence = 0.8, min_tracking_confidence = 0.5) as hands:
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        time_start = time.time()
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        
        frame.flags.writeable = False
        results = hands.process(frame)
        frame.flags.writeable = True
        
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        


        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                # Get landmarks
                index_tip = results.multi_hand_landmarks[0].landmark[8]
                thumb_tip = results.multi_hand_landmarks[0].landmark[4]
                pinky_tip = results.multi_hand_landmarks[0].landmark[20]

                # Draw landmarks
                mp_drawing.draw_landmarks(frame, hand, mp_hand.HAND_CONNECTIONS)
                distance = calculate_distance(index_tip.x,thumb_tip.x,index_tip.y,thumb_tip.y)
                x, y, z = index_tip.x, index_tip.y, index_tip.z

                y_before_filter = map_value(x, 0.1, 0.9, -20, 20)
                z_before_filter = map_value(y, 0.9, 0.1, 8, 25)
                x_before_filter = map_value(z, 0, -0.9, 20, 35)
                
                x_coppelia = low_pass_filter(prev_x, x_before_filter)
                y_coppelia = low_pass_filter(prev_y, y_before_filter)
                z_coppelia = low_pass_filter(prev_z, z_before_filter)

                prev_x = x_coppelia
                prev_y = y_coppelia
                prev_z = z_coppelia

                distance = calculate_distance(thumb_tip.x, index_tip.x, thumb_tip.y, index_tip.y)
                gripper_angle = map_angle(distance,0,0.8,90,20)

                distance_buzzer = calculate_distance(pinky_tip.x, thumb_tip.x, pinky_tip.y, thumb_tip.y)
                
                if distance_buzzer < 0.05:
                    movement_enabled = not movement_enabled
                    buzzer_feedback()
                    time.sleep(0.3)  # Debounce toggle gesture

        sim.simxSetObjectPosition(client_id, object_handle, -1, [x_coppelia/100, y_coppelia/100, z_coppelia/100], sim.simx_opmode_oneshot)

        angles = []
        return_codes = []

        for handle in joint_handles:
            return_code, angle = sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_buffer)

            if return_code == sim.simx_return_ok:
                angles.append(math.degrees(angle))
            else:
                angles.append(-1) #buffer not ready

        if -1 not in angles:
            servos[0].write(angles[0])
            servos[1].write(angles[2])
            servos[2].write(angles[1])
            servos[3].write(angles[0])
            servos[4].write(gripper_angle)

                
        cv2.imshow('Hand Tracking', frame)

        sleep_duration = timestep - (time.time() - time_start)
        if sleep_duration >= 0:
            time.sleep(timestep - (time.time() - time_start))

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
sim.simxFinish(client_id)
home_position()



