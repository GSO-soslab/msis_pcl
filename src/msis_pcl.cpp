#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ping360_msgs/SonarEcho.h>
#include <mvp_msgs/Float64Stamped.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <algorithm>

class ImageConverter
{
  //ROS Stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber echo_sub_;
  ros::Publisher pub_pcl;
  
  //Sensor Info
  bool stonefish_enabled;
  std::string sub_topic;
  std::string pub_topic;
  std::string frame_id;
  float range_min;
  float range_max;
  float number_of_bins;
  float angle_radians;
  float cos_angle_radians;
  float sin_angle_radians;

  //Image stuff
  int height;
  int width;
  cv::Mat prev;
  cv::Mat current;
  cv::Mat current_gray;
  cv::Mat diff;
  std::vector<uchar> middle_intense;
  std::vector<uchar> intensities;
  float angle;
  
public:
  //Constructor
  ImageConverter() : it_(nh_)
  {
    nh_.getParam("stonefish/enabled", stonefish_enabled);

    //If StoneFish
    if (stonefish_enabled == true){
      nh_.getParam("stonefish/sub_topic", sub_topic);
      nh_.getParam("stonefish/frame", frame_id);
      nh_.getParam("stonefish/range_min", range_min);
      nh_.getParam("stonefish/range_max", range_max);
      nh_.getParam("stonefish/number_of_bins", number_of_bins);
      nh_.getParam("stonefish/pub_topic", pub_topic);
      pub_pcl = nh_.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
      // Subscrive to input video feed
      image_sub_ = it_.subscribe(sub_topic, 1, &ImageConverter::imageCb, this);
      }

    //Else Ping360
    else{
      nh_.getParam("ping360/sub_topic", sub_topic);
      nh_.getParam("ping360/pub_topic", pub_topic);
      pub_pcl = nh_.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
      range_min = 0;
      nh_.getParam("ping360/frame", frame_id);
      //Sub to echo message
      echo_sub_ = nh_.subscribe(sub_topic, 1, &ImageConverter::echoCb, this);
    }
  }

  //Ping360 Callback
  void echoCb(const ping360_msgs::SonarEcho::Ptr& msg){
    this->angle_radians = msg->angle;
    this->cos_angle_radians =  std::cos(this->angle_radians);
    this->sin_angle_radians =  std::sin(this->angle_radians);
    this->intensities = msg->intensities;
    this->range_max = msg->range;

    if (this->range_max == 1){
        this->number_of_bins = 666;
      }
      else{
        this->number_of_bins = 1200;
      }
    sensor_msgs::PointCloud2 pcl_msg;
    
    //Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32);

    //Msg header
    pcl_msg.header = std_msgs::Header();
    pcl_msg.header.frame_id = this->frame_id;

    pcl_msg.height = 1;
    //No. of bins equal to the image width equal to the number of points
    pcl_msg.width = this->number_of_bins;
    pcl_msg.is_dense = true;
    pcl_msg.header.stamp = msg->header.stamp;
    //Total number of bytes per point
    pcl_msg.point_step = 16;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.width * pcl_msg.point_step);

    //x positions.
    std::vector<float> x = this->linspace(this->range_min, this->range_max, this->number_of_bins);
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

    for (size_t i = 0; i < pcl_msg.width; ++i) {
        // std::cout<<x[i]<<std::endl;
        *iterX = x[i] * this->cos_angle_radians;
        *iterY = x[i] * this->sin_angle_radians;
        *iterZ = 0;

        *iterIntensity = static_cast<uchar>(this->intensities[i]);

          // // Increment the iterators
          ++iterX;
          ++iterY;
          ++iterZ;
          ++iterIntensity;
    }
    this->pub_pcl.publish(pcl_msg);

    //transform and Publish it in odom frame
  }

  //Stonefish Image callback (image processing)
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //Cv Bridge
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   
    if(this->prev.empty()) {
      this->prev = cv_ptr->image;
      return;
    }

    this->current = cv_ptr->image;
    //Get current measurement
    cv::absdiff(this->prev, this->current, this->diff);

    cv::cvtColor(this->diff, this->diff, CV_BGR2GRAY);
    this->prev = this->current;
    
    cv::cvtColor(this->current, this->current_gray,CV_BGR2GRAY);
    cv::Size s=this->diff.size();
    this->height = s.height;
    this->width  = s.width;
    
    this->generate_pointclouds();
    
  }

  //Generate Pointclouds from image
  void generate_pointclouds(){

    sensor_msgs::PointCloud2 pcl_msg;
    
    //Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32);

    //Msg header
    pcl_msg.header = std_msgs::Header();
    pcl_msg.header.stamp = ros::Time::now();
    pcl_msg.header.frame_id = this->frame_id;

    pcl_msg.height = 1;
    //No. of bins equal to the image height equal to the number of points
    pcl_msg.width = this->height;
    pcl_msg.is_dense = true;

    //Total number of bytes per point
    pcl_msg.point_step = 16;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.width * pcl_msg.point_step);

    //x positions.
    std::vector<float> x = this->linspace(this->range_min, this->range_max, this->number_of_bins);
    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

    //Get the middle values
    this->middle_intense = this->getMiddleRowPixelValues(this->diff);
    for (int angle=0; angle < this->middle_intense.size();angle ++){
      //Get current angle measurement
      if (this->middle_intense[angle] != 0){
        this->angle = angle;
        this->cos_angle_radians = std::cos((this->angle* 2*M_PI / 400.0 - M_PI));
        this->sin_angle_radians = std::sin((this->angle* 2*M_PI / 400.0 - M_PI));
        // std::cout<<this->angle* 2*M_PI / 400.0 - M_PI<<std::endl;
        for (size_t i = 0; i < pcl_msg.width; ++i) {
          *iterX = x[i] * this->cos_angle_radians;
          *iterY = x[i] * this->sin_angle_radians;
          *iterZ = 0;

          this->intensities = this->getColumnPixelValues(this->current_gray, this->angle);

          *iterIntensity = static_cast<uchar>(this->intensities[i]);

          // printVector(this->intensities);

          // // Increment the iterators
          ++iterX;
          ++iterY;
          ++iterZ;
          ++iterIntensity;
        }
      }
    }

    this->pub_pcl.publish(pcl_msg);
  }

  //Linspace function
  std::vector<float> linspace(float start, float end, size_t points){
    std::vector<float> res(points);
    float step = (end - start) / (points - 1);
    size_t i = 0;
    for (auto& e : res)
    {
      e = start + step * i++;
    }
    return res;
  }

  //get middle row pixel values
  std::vector<uchar> getMiddleRowPixelValues(const cv::Mat& image) {
      std::vector<uchar> pixelValues;

      if (image.empty()) {
          std::cerr << "Empty image." << std::endl;
          return pixelValues;
      }

      if (image.channels() != 1) {
          std::cerr << "Image is not grayscale." << std::endl;
          return pixelValues;
      }

      int numRows = image.rows;
      int numCols = image.cols;

      int middleRow = numRows / 2;

      cv::Mat middleRowPixels = image.row(middleRow);

      for (int col = 0; col < numCols; ++col) {
          pixelValues.push_back(middleRowPixels.at<uchar>(col));
      }

      return pixelValues;
    }
  
  //get Column-wise pixel values.
  std::vector<uchar> getColumnPixelValues(const cv::Mat& image, int columnIndex) {
    // Check if the column index is within the image boundaries
    if (columnIndex < 0 || columnIndex >= image.cols) {
        throw std::out_of_range("Column index is out of range");
    }

    std::vector<uchar> pixelValues;
    pixelValues.reserve(image.rows);

    // Iterate over each row in the column and retrieve the pixel value
    for (int rowIndex = image.rows; rowIndex > 0; --rowIndex) {
        uchar pixelValue = image.at<uchar>(rowIndex, columnIndex);
        pixelValues.push_back(pixelValue);
    }
    // printVector(pixelValues);
    return pixelValues;
}

  //convert degrees to radians
  double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
  }

  //display a vector
  void printVector(const std::vector<uchar>& vec) {

    for (const auto& element : vec) {
        std::cout << static_cast<int>(element) << " ";
    }
    auto maxElement = std::max_element(vec.begin(), vec.end());
    if (maxElement != vec.end()) {
        std::cout << "The largest value is: " << static_cast<int>(*maxElement) << std::endl;
    }
    std::cout << std::endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msis_pcl_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}