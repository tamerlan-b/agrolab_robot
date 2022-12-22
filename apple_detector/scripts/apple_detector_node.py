#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageProcessor:
    def getContourCenters(contours):
        centers = []
        for c in contours:
            # compute the center of the contour
            M = cv2.moments(c)
            if M["m00"] == 0.0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))
        return centers

    def drawCenters(img, centers):
        for c in centers:
            cv2.circle(img, (c[0], c[1]), 7, (0, 0, 0), -1)
            text = f"(U,V) = ({c[0]}, {c[1]})"
            text_pos = (c[0] - 30, c[1] - 30)
            cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return img

    # Находит красные кубики на изображении с камеры и возвращает их центры
    def findObjects(img):
        # Переводим в HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Бинаризуем
        hsv_min = np.array((120,50,0), np.uint8)
        hsv_max = np.array((130,255,255), np.uint8)
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        # Детектируем контуры
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Вычисляем центры объектов
        centers = ImageProcessor.getContourCenters(contours)
        return centers

    def process_image(img):
        centers = ImageProcessor.findObjects(img)
        cnt_img = img.copy()
        cnt_img = ImageProcessor.drawCenters(cnt_img, centers)
        return cnt_img



class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/agrolabCamera/image_raw", Image, self.callback)

        self.obj_center_pub = rospy.Publisher('~detected_object', Point, queue_size=10)

        self.debug_img_pub = rospy.Publisher('~debug/image', Image, queue_size=10)


    def callback(self, img: Image):
        # Переводим изображение в OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        # Находим центры красных объектов
        centers = ImageProcessor.findObjects(cv_image)
        # print(centers)

        # Публикуем центры объектов в топик
        for c in centers:
            p = Point(x=c[0], y=c[1], z=0)
            self.obj_center_pub.publish(p)

        # Визуализируем найденные центры
        cnt_img = cv_image.copy()
        cnt_img = ImageProcessor.drawCenters(cnt_img, centers)
        debug_img_msg = self.bridge.cv2_to_imgmsg(cnt_img, encoding='rgb8', header=img.header)
        self.debug_img_pub.publish(debug_img_msg)
        # rospy.loginfo("I get image")
    
def main():    
    rospy.init_node('apple_detector', anonymous=True)

    rospy.loginfo("Apple detector started")

    cd = ColorDetector()

    rospy.spin()

if __name__ == '__main__':
    main()