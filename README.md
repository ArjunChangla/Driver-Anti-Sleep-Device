import cv2
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import os

# Setup GPIO
BUZZER_PIN = 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)


# Initialize PiCamera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

# Load OpenCV classifiers for face and eye detection
cascades_path='/home/pi/opencv-4.0.0/data/haarcascades'
face_cascade = cv2.CascadeClassifier(os.path.join(cascades_path, 'haarcascade_frontalface_default.xml'))
eye_cascade =  cv2.CascadeClassifier(os.path.join(cascades_path, 'haarcascade_eye.xml'))

#Threshold to trigger the buzzer
EYE_CLOSED_THRESHOLD=15

eye_closed_counter =0

def detect_drowsiness(frame):
global eye_closed_counter

     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     faces = face_cascade.detectMultiScale(gray, 1.3, 5)
   
     eyes_detected=False
   
    for (x, y, w, h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]
       
        eyes = eye_cascade.detectMultiScale(roi_gray)
       
        if len(eyes) > 0:
eyes_detected=True
eye_closed_counter=0

for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
               
         else:
eye_closed_counter +=1

if eye_closed_counter>= EYE_CLOSED_THRESHOLD:

            # No eyes detected, sound the buzzer
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
        else:
            # Eyes detected, turn off the buzzer
            GPIO.output(BUZZER_PIN, GPIO.LOW)

    return frame

try:
    # Capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        output = detect_drowsiness(image)
       
        # Display the resulting frame
        cv2.imshow("Frame", output)
       
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
       
        if key == ord("q"):
            break

finally:
    # Cleanup
    GPIO.cleanup()
    cv2.destroyAllWindows()
