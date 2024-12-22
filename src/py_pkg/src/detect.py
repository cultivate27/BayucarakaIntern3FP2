#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import cv2
from yolov5 import YOLOv5

model = YOLOv5("models/exp8/weights/best.pt")
rospy.loginfo("YOLOv5 model loaded successfully!")

cap = cv2.VideoCapture(2)

def detect_and_publish():
    pub = rospy.Publisher('object_coordinates', Float32MultiArray, queue_size=10)
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        results = model.predict(frame)

        coordinates = Float32MultiArray()
        
        for *box, conf, cls in results.xywh[0]:
            x_center, y_center, width, height = box
            coord_x = x_center.item()
            coord_y = y_center.item()
            coord_z = 0.0  

            rospy.loginfo(f"X: {coord_x:.2f}, Y: {coord_y:.2f}")
            coordinates.data = [coord_x, coord_y, coord_z]
            pub.publish(coordinates)

        rate.sleep()

if __name__ == '__main__':
    try:
        detect_and_publish()
    except rospy.ROSInterruptException:
        pass