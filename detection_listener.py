#!/usr/bin/env python3

import rospy
from trasnform_frame import transform_point
from object_detection_msgs.msg import ObjectDetectionInfoArray
import pandas as pd

class ArtifactReport():
    def __init__(self):
        self.df = pd.DataFrame(columns=['class_id', 'x', 'y', 'z'])
        self.tollerance = 1

    # Callback function
    def update_df(self, data):
        
        for detection in data.info:
            
            #rospy.loginfo("Class ID: %s", detection.class_id)
            #rospy.loginfo("Camera Frame: x=%f, y=%f, z=%f", detection.position.x, detection.position.y, detection.position.z)
            #rospy.loginfo(detection)
            

            # Transform coordinates
            transformed_point = transform_point(detection.position, "rgb_camera_optical_link", "world_graph_msf")
            #rospy.loginfo("World Frame: x=%f, y=%f, z=%f", transformed_point.x, transformed_point.y, transformed_point.z)

            # Check if already detected
            check_if_detected = False
            if check_if_detected:
                for index, row in self.df.iterrows():
                    if row['class_id'] == detection.class_id:
                        if abs(row['x'] - transformed_point.x) < self.tollerance and abs(row['y'] - transformed_point.y) < self.tollerance and abs(row['z'] - transformed_point.z) < self.tollerance:
                            #rospy.loginfo("Already detected")
                            return
    
        

            # Add the new entry
            new_entry = {'class_id': detection.class_id, 'x': transformed_point.x, 'y': transformed_point.y, 'z': transformed_point.z}
            self.df = pd.concat([self.df, pd.DataFrame([new_entry])], ignore_index=True)

        rospy.loginfo("--------------START-----------")
        rospy.loginfo(self.df)
        rospy.loginfo("--------------END-----------")

        rospy.loginfo("Done with Loop")
        rospy.loginfo(self.df)



    def export_csv(self):
        self.df.to_csv('./src/extract_artifacts/detected_artifacts/detected_artifacts.csv', index=False)




def listener():

    # Init node
    rospy.init_node('detection_listener', anonymous=True)

    # Create df
    artifact_report = ArtifactReport()
    rospy.Rate(5)

    # Subscribe to the detection info topic
    rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, artifact_report.update_df)
    rospy.spin()

    artifact_report.export_csv()

if __name__ == '__main__':
    listener()
