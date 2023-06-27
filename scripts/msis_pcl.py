#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import numpy as np
from cv_bridge import CvBridge
import cv2

class MSIS_PCL:
    def __init__(self) -> None:
        #Publisher info
        pub_topic = rospy.get_param("pub_topic","/msis/pointcloud/python")
        self.pub_pcl = rospy.Publisher(pub_topic, PointCloud2, queue_size=1)
        
        self.pointcloud_msg = PointCloud2()
        self.pointcloud_msg.header = Header()
        
        self.stonefish_enabled = rospy.get_param("stonefish/enabled")

        #Get stonefish details
        if self.stonefish_enabled:
            sub_topic = rospy.get_param("stonefish/sub_topic")                    
            self.range_min= rospy.get_param("stonefish/range_min")
            self.range_max= rospy.get_param("stonefish/range_max")
            self.number_of_bins = rospy.get_param("stonefish/number of bins")
            self.pointcloud_msg.header.frame_id = rospy.get_param("stonefish/frame")
            self.prev = np.zeros((self.number_of_bins,360)).astype(np.float32)
            self.current = np.zeros((self.number_of_bins,360)).astype(np.float32)
        else:
        #or ping360 sensor details
            sub_topic = rospy.get_param("ping360/sub_topic", "/image")
            self.range_min = 0.75 #Technical Documentation
            self.range_max = rospy.get_param("/ping360_sonar_node/Configuration/range")
            self.number_of_bins = round(rospy.get_param("/ping360_sonar_node/Driver/image_size")/2)
            self.pointcloud_msg.header.frame_id = rospy.get_param("/ping360_sonar_node/Driver/frame_id")
            self.prev = np.zeros((self.number_of_bins *2,self.number_of_bins*2)).astype(np.float32)
            self.current = np.zeros((self.number_of_bins*2,self.number_of_bins*2)).astype(np.float32)

        rospy.Subscriber(sub_topic, Image, self.image_Cb, queue_size=1)
        
        # Define the point fields (attributes)
        fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32,1),
        # PointField('rgb', 16, PointField.INT32,1),
        ]
        self.pointcloud_msg.fields = fields

        # Set the point cloud dimensions (width and height)
        self.pointcloud_msg.width = self.number_of_bins #HOW MANY POINTS in total (range)
        self.pointcloud_msg.height = 1
        self.pointcloud_msg.point_step = 4 * (len(fields))  # Each point occupies 16 bytes

        # Populate the point data
        num_points = self.pointcloud_msg.width * self.pointcloud_msg.height
        self.pointcloud_msg.row_step = self.pointcloud_msg.point_step * num_points
        self.pointcloud_msg.is_dense = True  # All points are valid

        self.x = np.linspace(self.range_min,self.range_max,self.number_of_bins)
        #height is number of bins ie no of points. 4 attributes :x,y,z,intensity
        self.points = np.zeros((self.number_of_bins,len(fields)), dtype=np.float32)

    def image_Cb(self, msg):
        bridge = CvBridge()
        #GrayScale Image with Bins x 360
        self.current = bridge.imgmsg_to_cv2(msg)
        self.diff = (self.prev - self.current)
        self.height, self.width = self.diff.shape
        self.prev = self.current
        if self.stonefish_enabled:
            self.generate_pointclouds_stonefish()
        else:
            self.generate_pointclouds_ping360()

    def generate_pointclouds_ping360(self):
        #generating pointcloud using the radial image.
        self.show_current_measurement()

        
    def generate_pointclouds_stonefish(self):
        #Generating pointcloud using the rectangular image.
        row_values = self.get_middle_row_intensities(self.diff)
        for angle in range(len(row_values)):
            if row_values[angle]!=0:
                theta = 180-angle
                for i in range(len(self.x)): #number of bins
                    # Add X values
                    self.points[:][i][0] = self.x[i] * np.cos(np.deg2rad(theta))
                    
                    # Add Y Value
                    self.points[:][i][1] = self.x[i] * np.sin(np.deg2rad(theta))
                    
                    # Add intensities
                    column_intensities = self.get_column_intensities(self.current, angle)
                    self.points[:][i][3] = column_intensities[i]

                    # Add rgb
                    # points[:][i][4] = column_intensities[i]
                # Convert the points data to a byte string and assign it to the point cloud message
                self.pointcloud_msg.data = self.points.tobytes()
                self.pointcloud_msg.header.stamp = rospy.Time.now()
                
                self.pub_pcl.publish(self.pointcloud_msg)

    def get_middle_row_intensities(self,image):
        # Get the dimensions of the image
        # Calculate the index of the middle column
        middle_row_index = self.height // 2
        # Retrieve the intensities from the middle row
        middle_row_intensities = image[middle_row_index, :]
        return middle_row_intensities
    
    def get_column_intensities(self, image, column_index):
        # Retrieve the intensities from the specified column
        column_intensities = image[:, column_index]
        return np.flip(column_intensities) 
    
    def show_current_measurement(self):
        cv2.imshow("window",self.diff)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("MSIS_to_PCL")
    MSIS_PCL()
    rospy.spin()