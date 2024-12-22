#!/usr/bin/env python3
import rospy
import cv2
from yolov5 import YOLOv5
from cv_bridge import CvBridge

class YOLOv5LiveDetection:
    def __init__(self):
        rospy.init_node('debug', anonymous=True)

        self.model = YOLOv5("models/exp8/weights/best.pt")
        rospy.loginfo("YOLOv5 model loaded successfully!")

        self.capture = cv2.VideoCapture(2)
        self.bridge = CvBridge()

    def calculate_center(self, row):
        x_min, y_min, x_max, y_max = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        z_center = 1.0
        return x_center, y_center, z_center

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if not ret:
                rospy.logerr("Failed to capture frame from webcam")
                break

            results = self.model.predict(frame)
            detections = results.pandas().xyxy[0]
            rospy.loginfo(f"Detections: {len(detections)} objects detected")

            for _, row in detections.iterrows():
                x_center, y_center, z_center = self.calculate_center(row)
                rospy.loginfo(f"Object: {row['name']}, Center: (x={x_center:.2f}, y={y_center:.2f}, z={z_center:.2f})")

           
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("Shutting down node...")
                break
            
            rospy.loginfo("Loop iteration complete")
            rate.sleep()

        self.capture.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        detector = YOLOv5LiveDetection()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLOv5 Live Detection node terminated.")
