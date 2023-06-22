#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageToCostmap:
    def __init__(self) -> None:
        self.map_publisher = rospy.Publisher('/my_occupancy_grid', OccupancyGrid, queue_size=10)
        rospy.Subscriber("/wamv/msis/stonefish/data/display", Image, self.image_Cb, queue_size=1)
        
        self.bridge = CvBridge()

        self.number_of_bins = 100
        self.range_min = 0.5
        self.range_max= 50.0

    def image_Cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = cv2.flip(image, 1)
        image = np.round(image / 255.0 * 100).astype(np.uint8)
        self.height, self.width = image.shape
        

        self.generate_map(image)
    
    def generate_map(self, image):
        # Create an instance of the OccupancyGrid message
        occupancy_grid = OccupancyGrid()

        # Set the necessary fields of the message
        occupancy_grid.header.frame_id = 'wamv/msis'  # Frame ID for the occupancy grid
        occupancy_grid.info.resolution = 1
        # print(f"Height : {self.height}, Widht: {self.width}, Res: {occupancy_grid.info.resolution}")
        occupancy_grid.info.width =  self.width  # Width of the grid in cells
        occupancy_grid.info.height = self.height  # Height of the grid in cells
        occupancy_grid.info.origin.position.x = -round(occupancy_grid.info.height/2)
        occupancy_grid.info.origin.position.y = -round(occupancy_grid.info.width/2)


        # Populate the occupancy data (array of cell values)
        # Here, we assume the occupancy data is a 1D list of cell values
        occupancy_data = image.flatten().astype(int).tolist()#image.flatten()
        # print()
        occupancy_grid.data = occupancy_data #* occupancy_grid.info.height * occupancy_grid.info.height

        # Publish the occupancy grid
        publisher = rospy.Publisher('/my_occupancy_grid', OccupancyGrid, queue_size=10)
        rate = rospy.Rate(10)  # Adjust the publishing rate as needed

        occupancy_grid.header.stamp = rospy.Time.now()  # Update the timestamp
        publisher.publish(occupancy_grid)
        # print("here")
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('map_node')
    ImageToCostmap()
    rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('occupancy_grid_publisher')

#     # Create an instance of the OccupancyGrid message
#     occupancy_grid = OccupancyGrid()

#     # Set the necessary fields of the message
#     occupancy_grid.header.frame_id = 'map'  # Frame ID for the occupancy grid
#     occupancy_grid.info.resolution = 1  # Resolution of the grid cells
#     occupancy_grid.info.width = 10  # Width of the grid in cells
#     occupancy_grid.info.height = 10  # Height of the grid in cells
#     occupancy_grid.info.origin.position.x =0
#     occupancy_grid.info.origin.position.y =0


#     # Populate the occupancy data (array of cell values)
#     # Here, we assume the occupancy data is a 1D list of cell values
#     occupancy_data = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#     occupancy_grid.data = occupancy_data * 10

#     # Publish the occupancy grid
#     publisher = rospy.Publisher('/my_occupancy_grid', OccupancyGrid, queue_size=10)
#     rate = rospy.Rate(10)  # Adjust the publishing rate as needed

#     while not rospy.is_shutdown():
#         occupancy_grid.header.stamp = rospy.Time.now()  # Update the timestamp
#         publisher.publish(occupancy_grid)
#         # print("here")
#         rate.sleep()