#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class ImageConverter
{
  //ROS Stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub_pcl =nh_.advertise<sensor_msgs::PointCloud2>("msis/pointcloud", 1);
  
  //Sensor Info
  float range_min = 0.5;
  float range_max = 50.0;
  float number_of_bins = 100;
  
  //Image stuff
  int height;
  int width;
  cv::Mat prev = cv::Mat::zeros(this->number_of_bins, 360, CV_8UC1);
  cv::Mat last_img;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed
    image_sub_ = it_.subscribe("/wamv/msis/stonefish/data/image", 1, &ImageConverter::imageCb, this);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   
    if(last_img.empty()) {
      last_img = cv_ptr->image;
      return;
    }

    cv::Mat diff;
    cv::Mat curr_img = cv_ptr->image;
    cv::absdiff(last_img, curr_img, diff);
    last_img = curr_img;
    cv::Size s=diff.size();
    this->height = s.height;
    this->width  = s.width;
    
    this->generate_pointclouds();
  }

  void generate_pointclouds(){
    sensor_msgs::PointCloud2 pcl_msg;
    pcl_msg.header = std_msgs::Header();
    pcl_msg.header.stamp = ros::Time::now();
    pcl_msg.header.frame_id = "wamv/msis";

    pcl_msg.height = 1;
    pcl_msg.width = 1;//this->height;

    sensor_msgs::PointField  fieldx, fieldy, fieldz;
    fieldx.name = "x";
    fieldx.offset = 0;
    fieldx.datatype = sensor_msgs::PointField::FLOAT32;
    fieldx.count= 1;

    fieldy.name="y";
    fieldy.offset=4;
    fieldy.datatype=sensor_msgs::PointField::FLOAT32;
    fieldy.count = 1;

    fieldz.name="z";
    fieldz.offset=8;
    fieldz.offset=sensor_msgs::PointField::FLOAT32;
    fieldz.count=1;

    pcl_msg.fields.push_back(fieldx);
    pcl_msg.fields.push_back(fieldy);
    pcl_msg.fields.push_back(fieldz);

    pcl_msg.point_step = 12;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.width * pcl_msg.point_step);

        
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");

    for (size_t i = 0; i < pcl_msg.width; ++i) {
        // Set the coordinates of each point
        *iterX = 1;
        *iterY = 1;
        *iterZ = 0;

        // Increment the iterators
        ++iterX;
        ++iterY;
        ++iterZ;
    }
    this->pub_pcl.publish(pcl_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msis_pcl_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}