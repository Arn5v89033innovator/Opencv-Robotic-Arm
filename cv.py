import cv2
import mediapipe as mp
import serial
import time
import math

# Connect to Arduino
arduino = serial.Serial('COM5', 9600)  # Change COM port
time.sleep(2)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

def map_range(val, in_min, in_max, out_min, out_max):
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    success, img = cap.read()
    if not success:
        break

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Get coordinates
            h, w, _ = img.shape
            cx = int(hand_landmarks.landmark[0].x * w)  # Wrist X
            cy = int(hand_landmarks.landmark[0].y * h)  # Wrist Y

            # Map wrist position to shoulder & elbow angles
            shoulder_angle = map_range(cx, 0, w, 0, 180)
            elbow_angle = map_range(cy, 0, h, 0, 180)

            # Distance between thumb tip (4) & index tip (8) → Gripper
            thumb_tip = hand_landmarks.landmark[4]
            index_tip = hand_landmarks.landmark[8]
            dist = math.hypot((index_tip.x - thumb_tip.x) * w, (index_tip.y - thumb_tip.y) * h)

            # Map distance to gripper angle (smaller distance → closed)
            gripper_angle = int(map_range(dist, 20, 200, 90, 50))  
            gripper_angle = max(min(gripper_angle, 90), 50)  # Limit range

            # Send to Arduino
            data = f"S{shoulder_angle}E{elbow_angle}G{gripper_angle}\n"
            arduino.write(data.encode())

            # Debug info
            cv2.putText(img, f"Shoulder: {shoulder_angle}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(img, f"Elbow: {elbow_angle}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            cv2.putText(img, f"Gripper: {gripper_angle}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    cv2.imshow("Gesture Control Arm", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
