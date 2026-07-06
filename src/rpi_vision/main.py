import cv2
from flask import Flask, render_template, Response, request
from picamera2 import Picamera2
from PIL import Image
import time
import threading
from adafruit_servokit import ServoKit
import sys
import serial
import numpy as np

kit = ServoKit(channels=16)

min_angle0 = 35
max_angle0 = 145
min_angle1 = 50
max_angle1 = 90

mx = 21.33   #половина изображения разделить на половину угла объектива
my = 8
anglex = 90
angley = 80

def servo_write(num, angle):
    if num == 1:
        max_angle = max_angle1
        min_angle = min_angle1
    elif num == 0:
        max_angle = max_angle0
        min_angle = min_angle0
    
    if angle < min_angle:
        angle = min_angle
    elif angle > max_angle:
        angle = max_angle
    
    kit.servo[num].angle = angle


servo_write(0, 90)
servo_write(1, 80)

SERIAL_PORT = '/dev/serial0'  # сериал порт 0 для uart на gpio
BAUD_RATE = 115200

# инициализация подключения по uart
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit(1)

# подключение камеры через библиотеку picam2
picam2 = Picamera2()
config = picam2.create_preview_configuration({"size":(640, 480)})
picam2.configure(config)
picam2.start()
#picam2.set_controls({"ExposureTime": 000}) # возможность выставить время экспозиции кадра для большего fps и обнаружения только очень ярких меток
#time.sleep(1)
app = Flask(__name__)

curr_time = time.time()

img_to_send = picam2.capture_array()

stop_flag = True

human_detected = 0

# основная рабочая функция
def main():
    global img_to_send
    global m
    global human_detected
    frame_timer = 0
    global anglex
    anglex = 90
    angley = 75
    timer_nohuman_flag = False
    timer_nohuman = time.time()
    while stop_flag:
        fps = 1 / (time.time() - frame_timer)
        frame_timer = time.time() 

        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        img = cv2.rotate(img, cv2.ROTATE_180)
        
        mask = cv2.inRange(img, (0, 0, 248), (179, 15, 255))
        contours_unfiltered, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        img = cv2.putText(img, f'FPS: {fps}', (550, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'X: {anglex}', (550, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Y: {angley}', (550, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.drawContours(img, contours_unfiltered, -1, (0, 255, 0), 2)
    

        if len(contours_unfiltered) > 0:
            contours = []
            for i in contours_unfiltered:
                if not (20 < cv2.contourArea(i) < 3000):
                    continue

                if not cv2.isContourConvex(cv2.approxPolyDP(i, 3, True)):
                    continue
                
                contours.append(i)

            if len(contours) > 1:
                human_detected = 1
                timer_nohuman_flag = False

                contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[:4:]
                
                if len(contours) >= 4:
                    contours = contours[:4:]
                contours = sorted(contours, key=lambda i: cv2.boundingRect(i)[0])  
                centers_x, centers_y = [], []
                for i in range(len(contours)):
                    moments = cv2.moments(contours[i])

                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    centers_x.append(cx)
                    centers_y.append(cy)
                    cv2.line(img, (cx, cy), (centers_x[i - 1], centers_y[i - 1]), (255, 0, 0), 2)
                    cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                    img = cv2.putText(img, f'{cx, cy}, {int(cv2.contourArea(contours[i]))}', (cx + 10, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

                if len(contours) == 4:
                    cv2.line(img, (centers_x[0], centers_y[0]), (centers_x[2], centers_y[2]), (255, 0, 0), 2)
                    cv2.line(img, (centers_x[1], centers_y[1]), (centers_x[3], centers_y[3]), (255, 0, 0), 2)

                print(centers_x, centers_y, sep=' ')
                center_x = int(sum(centers_x) / len(centers_x))
                center_y = int(sum(centers_y) / len(centers_y))

                cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
                if abs(center_x - 320) > 5:
                    anglex -= (center_x - 320) / (mx * 1.75)
                    anglex = round(anglex, 2)
                    if anglex < min_angle0:
                        anglex = min_angle0
                    if anglex > max_angle0:
                        anglex = max_angle0

                    servo_write(0, anglex)

                if abs(center_y - 240) > 5:
                    angley -= (center_y - 240) / (my * 3.5)
                    angley = round(angley, 2)
                    if angley < min_angle1:
                        angley = min_angle1
                    if angley > max_angle1:
                        angley = max_angle1
                    servo_write(1, angley)
            else:
                if not timer_nohuman_flag:
                    timer_nohuman = time.time()
                    timer_nohuman_flag = True
                elif time.time() - timer_nohuman > 1:
                    human_detected = 0
                    anglex = 90
                    angley = 80
                    servo_write(0, anglex)
                    servo_write(1, angley)
                    

        else:
            if not timer_nohuman_flag:
                timer_nohuman = time.time()
                timer_nohuman_flag = True
            elif time.time() - timer_nohuman > 1:
                human_detected = 0
                anglex = 90
                angley = 80
                servo_write(0, anglex)
                servo_write(1, angley)
                    
        
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 1)
        cv2.line(img, (320, 0), (320, 480), (0, 0, 0), 1)
        img_to_send = img

# функция отправки данных по uart
def send_uart():
    global ser
    global anglex
    old_data = None
    while stop_flag:
        data = str(anglex) + ' ' + str(human_detected)
        if data != old_data:
            ser.write((data + '\n').encode('utf-8'))
            print("Message sent:", data)
            old_data = data
        time.sleep(0.01)
    ser.close()
    print('Serial port is closed.')

# функция преобразования кадров
def getFramesGenerator():
    global curr_time
    global img_to_send
    while stop_flag:
        img_to_send = cv2.resize(img_to_send, (640, 480), interpolation=cv2.INTER_AREA)
        _, buffer = cv2.imencode('.jpg', img_to_send)
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    img_to_send = cv2.putText(img_to_send, "-CAMERA OFFLINE-", (10, 255), cv2.FONT_HERSHEY_TRIPLEX, 1.85, (255, 200, 200), 3)
    _, buffer = cv2.imencode('.jpg', img_to_send)
    yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


# функции для веб-страницы
@app.route('/video_feed')
def video_feed():
    return Response(getFramesGenerator(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')


main_thread = threading.Thread(target=main)
main_thread.start()

uart_thread = threading.Thread(target=send_uart)
uart_thread.start()

app.run(debug=False, host='10.42.0.1', port='5000')

try:
    while True:
        time.sleep(0.1)

# обработка исключения в случае прерывания работы программы через ctrl+c
except KeyboardInterrupt:
        kit.servo[0].angle = 90
        kit.servo[1].angle = 90
        time.sleep(0.3)
        kit.servo[0].angle = None
        kit.servo[1].angle = None
        print("Program finished due to keyboard interrupt.")
        stop_flag = False
        main_thread.join()
        uart_thread.join()