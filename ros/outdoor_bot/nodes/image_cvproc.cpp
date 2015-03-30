#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
//#include "outdoor_bot/digital_cams_custom.h"

using namespace cv;
using namespace std;

class image_cvproc
{
private:
   ros::NodeHandle nh_;
   ros::Publisher filename_pub_;
   std::string camFilename_;  
   image_transport::ImageTransport it_;
   image_transport::Publisher image_pub_;
   image_transport::Subscriber subWebcam_;
   image_transport::Subscriber subDigcam_;
   ros::Subscriber digcam_file_sub_;
   ros::Subscriber webcam_file_sub_;
   Mat fileImage_;

public:
   image_cvproc(ros::NodeHandle &nh)
   :  nh_(nh), it_(nh)
   {
      filename_pub_ = nh_.advertise<std_msgs::String>("camera_file", 50); // advertise camera files
      image_pub_ = it_.advertise("camera_image", 1); 
      subDigcam_ = it_.subscribe("digcam_image", 1, &image_cvproc::imageCallback, this);
      subWebcam_ = it_.subscribe("webcam_image", 1, &image_cvproc::imageCallback, this);
      webcam_file_sub_ = nh.subscribe("webcam_file", 50, &image_cvproc::cameraFileCallback, this); 
      digcam_file_sub_ = nh.subscribe("digcam_file", 50, &image_cvproc::cameraFileCallback, this);
   }

   void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("image received");
     try
     {
       cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
       cv::waitKey(30);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }

   void cameraFileCallback(const std_msgs::String::ConstPtr& msg)
   {
      ROS_INFO("image filename received");
      camFilename_ = msg->data.c_str();  
   }

   bool readImageFile(std::string filename)
   {
      fileImage_ = imread(filename, CV_LOAD_IMAGE_UNCHANGED); 
      //cv::waitKey(30);
      if (fileImage_.empty()) //check whether the image is loaded or not
      {
          ROS_ERROR("Error : readImageFile failed to read an image");
          return false;
      }
      return true;

   }

   bool writeToFile(std::string filename, Mat image)
   {
      if (image.empty()) return false;
      vector<int> compression_params; //vector that stores the compression parameters of the image
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
      compression_params.push_back(98); //specify the compression quality
      bool bSuccess = imwrite(filename, image, compression_params); //write the image to file
      if (bSuccess) 
      {
         std_msgs::String msg;
         msg.data = filename;
         filename_pub_.publish(msg);   // publish the filename
         ROS_INFO("image written to file");
         return true;
      }
      else ROS_ERROR("error in writing image to file");
      return false;
   }

   void publishImage(Mat image)
   {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      image_pub_.publish(msg);
   }
};

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "image_processor");
   ros::NodeHandle nh;

   image_cvproc ic(nh);

   cv::namedWindow("view");
   cv::startWindowThread();

/*
   cv_bridge::CvImagePtr cv_ptr;   // CvImagePtr is an opencv format 
   // might need the headers, don't know, otherwise where do these parameters come from?
   //msg->encoding = "bgr8";
   //msg->height = 200;
   //msg->width = 200;

   try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // converts from ROs to opencv format, makes a copy so we can alter it
   } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
   }

   cv::Mat &mat = cv_ptr->image;  // we can get the opencv Mat format from the CvImage format
    
   // we can get the ROS image publishing formatfrom it too
   sensor_msgs::ImagePtr imageForPublishingROS = cv_ptr->toImageMsg();

   pub.publish(imageForPublishingROS);
*/
   ros::spin();
   cv::destroyWindow("view");

   return EXIT_SUCCESS;
}
