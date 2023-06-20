#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import numpy as np
from cv_bridge import CvBridge

class MSIS_PCL:
    def __init__(self) -> None:
        self.pub_pcl = rospy.Publisher("/msis/pointcloud", PointCloud2, queue_size=1)
        rospy.Subscriber("/wamv/msis/stonefish/data/image", Image, self.image_Cb, queue_size=1)
        self.range_min= 0.5
        self.range_max= 50.0
        self.number_of_bins = 100
        self.prev = np.zeros((self.number_of_bins,360)).astype(np.float32)
        self.current = np.zeros((self.number_of_bins,360)).astype(np.float32)

        self.pointcloud_msg = PointCloud2()
        # Set the header information
        self.pointcloud_msg.header = Header()
        self.pointcloud_msg.header.stamp = rospy.Time.now()
        self.pointcloud_msg.header.frame_id = "wamv/msis"

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
        self.generate_pointclouds()

    def generate_pointclouds(self):
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

if __name__ == "__main__":
    rospy.init_node("MSIS_to_PCL")
    MSIS_PCL()
    rospy.spin()