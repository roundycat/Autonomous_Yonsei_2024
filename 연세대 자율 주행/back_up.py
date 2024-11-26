#!/usr/bin/env python
# -*- coding: utf-8 -*- 
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge, CvBridgeError

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 20  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
ultra_msg = None  # 초음파 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
View_Center = WIDTH // 2  # View의 중앙값
angle_adjustment = 0  # 조향각 보정을 위한 변수

#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.flip(image, 0)  # 이미지 뒤집기
    except CvBridgeError as e:
        rospy.logerr(e)
        print(e)

#=============================================
# 콜백함수 - 초음파 토픽을 받아서 처리하는 콜백함수
#=============================================
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받는다.
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)

#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 모터로 토픽을 보내는 함수
#=============================================
def sonic_drive():
    global new_angle, new_speed

    if ultra_msg is None:
        return

    if ultra_msg[1] > ultra_msg[3]:  # 왼쪽 벽이 오른쪽 벽보다 멀리 있으면, 왼쪽으로 주행
        new_angle = -50
    elif ultra_msg[1] < ultra_msg[3]:  # 왼쪽 벽보다 오른쪽 벽이 멀리 있으면, 오른쪽으로 주행
        new_angle = 50
    else:  # 위 조건들에 해당하지 않는 경우라면 직진 주행
        new_angle = 0

    new_speed = Fix_Speed
    drive(new_angle, new_speed)

#=============================================
# 정지선이 있는지 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_stopline():
    global image

    if image.size == 0:
        return False

    roi = image[HEIGHT//2:HEIGHT, WIDTH//3:2*WIDTH//3]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    _, _, v = cv2.split(hsv)
    _, binary = cv2.threshold(v, 200, 255, cv2.THRESH_BINARY)

    stopline_count = cv2.countNonZero(binary)
    
    if stopline_count > 1000:  # 임계값, 조정 가능
        return True
    else:
        return False
    
#=============================================
# 카메라 이미지에서 차선을 찾아 그 위치를 반환하는 함수
#=============================================
def lane_detect():
    global image

    if image.size == 0:
        return False, 0, 0

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 이미지의 가운데 부분만 선택 (가로와 세로 모두)
    height, width = hsv.shape[:2]
    middle_width_start = int(width * 0.2)  # 가로의 30%에서 시작
    middle_width_end = int(width * 0.8)  # 가로의 70%에서 끝
    middle_height_start = int(height * 0.1)  # 세로의 30%에서 시작
    middle_height_end = int(height * 0.5)  # 세로의 70%에서 끝
    middle_roi = hsv[middle_height_start:middle_height_end, middle_width_start:middle_width_end]

    # 검출 범위를 검은색 사각형으로 표시
    cv2.rectangle(image, (middle_width_start, middle_height_start), (middle_width_end, middle_height_end), (0, 0, 0), 2)

    # 노란색 검출
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(middle_roi, yellow_lower, yellow_upper)
    yellow_edges = cv2.Canny(yellow_mask, 50, 150)
    yellow_lines = cv2.HoughLinesP(yellow_edges, 1, np.pi / 170, 10, minLineLength=5, maxLineGap=8)

    # 흰색 검출
    white_lower = np.array([0, 0, 190])
    white_upper = np.array([255, 255, 255])
    white_mask = cv2.inRange(middle_roi, white_lower, white_upper)
    white_edges = cv2.Canny(white_mask, 30, 150)
    white_lines = cv2.HoughLinesP(white_edges, 1, np.pi / 180, 10, minLineLength=5, maxLineGap=5)

    # 왼쪽 및 오른쪽 차선 점 리스트 초기화
    left_points = []
    right_points = []

    # 검출된 노란색 및 흰색 차선의 점 분류
    def classify_points(lines):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 이미지의 가운데 영역에서의 좌표로 변환
                x1 += middle_width_start
                x2 += middle_width_start
                y1 += middle_height_start
                y2 += middle_height_start
                slope = (y2 - y1) / (x2 - x1)
                if slope < -0.5:
                    left_points.append((x1, y1))
                    left_points.append((x2, y2))
                elif slope > 0.5:
                    right_points.append((x1, y1))
                    right_points.append((x2, y2))
    
    classify_points(yellow_lines)
    classify_points(white_lines)

    if len(left_points) == 0 or len(right_points) == 0:
        return False, 0, 0

    left_points = np.array(left_points)
    right_points = np.array(right_points)

    x_left = int(np.mean(left_points[:, 0]))
    x_right = int(np.mean(right_points[:, 0]))
    
    # 점을 이미지에 그리기
    for x, y in left_points:
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)  # 좌측 차선을 녹색 점으로 표시
    for x, y in right_points:
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # 우측 차선을 빨간 점으로 표시

    return True, x_left, x_right


#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor, ultra_msg, image 
    global new_angle, new_speed, angle_adjustment
   
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")
    
    drive(0, 0)
    time.sleep(5)
    
    cam_exposure(100)
    filepath = "/home/pi/desktop/image"
    i = 0
    while not rospy.is_shutdown():
        sonic_drive()  # 초음파 센서를 이용한 주행
        
        found, x_left, x_right = lane_detect()  # 차선 인식
            
        if found:
            x_midpoint = (x_left + x_right) // 2 
            new_angle = (x_midpoint - View_Center) / 2 + angle_adjustment
            angle_adjustment = 0  # 각도 보정 초기화
        else:
            new_angle = angle_adjustment  # 차선을 인식하지 못하면 보정된 각도로 주행

        new_speed = Fix_Speed
        drive(new_angle, new_speed)  
        print(f"Angle: {new_angle}, Speed: {new_speed}")
    
        #cv2.imshow('Lane Detection', image)
		#height, _, = image.shape
		#save_image = image[int(height / 2):,:,:]
		#save_image = cv2.cvtColor(save_image, cv2.COLOR_BGR2YUV)
		#save_image = cv2.GaussiaBlur(save_image, (3,3), 0)
		#save_image = cv2.resize(save_image, (200,66))
        cv2.imshow("save", save_image)
        #cv2.imwrite("/home/pi/desktop/image/test.png", save_image)
        
        if new_angle >= 45:
            cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 45), image)
        elif new_angle <= -45:
            cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, -45), image)
        else:
            cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 0), image)
        #cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 45), image)
        i = i + 1
        cv2.waitKey(1)

        time.sleep(0.1)  # 주기를 줄여서 더 부드러운 주행을 위해

if __name__ == '__main__':
    start()

