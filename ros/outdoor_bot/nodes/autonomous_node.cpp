#include <ros/ros.h>
#include "outdoor_bot/webcams_custom.h"
#include "outdoor_bot/digcams_service.h"
#include "outdoor_bot/digcams_custom.h"
#include "outdoor_bot/NavTargets_service.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/mainTargets_msg.h"
#include "std_msgs/String.h"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <time.h>

using namespace std;

#define FRONT_CAMERA_NUMBER 0
#define REAR_CAMERA_NUMBER 1
#define PHASE_TWO_TIME_LIMIT 7200   //  7200 seconds in two hours-- phase two challenge
#define PHASE_ONE_TIME_LIMIT 1800 // 1800 seconds in 30 min-- phase one challenge
#define WEBCAM 0
#define DIGCAM 1

ros::ServiceClient NavTargets_client_, mainTargets_client_, digcams_client_;
ros::Publisher digcam_pub_, motion_pub_, webcam_pub_;
ros::Subscriber home_center_;
outdoor_bot::webcams_custom webcam_command_;
outdoor_bot::digcams_custom digcam_command_; 
int centerX_, centerY_, totalX_;
int home_image_height_, home_image_width_;
bool homeCenterUpdated_;
float range_, approxRangeToTarget_;
bool zoomResult_, writeFileResult_, newMainTargetImageReceived_, rangeUnknown_;
int retCapToMemory_;


int getUserInputInteger()
{
   int myNumber = 0;
   while (true)
   {
      string input = "";
      cout << "Please enter a valid number: " << endl << endl;
      getline(cin, input);

      // This code converts from string to number safely.
      stringstream myStream(input);
      if (myStream >> myNumber)
        break;
      cout << "Invalid number, please try again" << endl;
   }
   cout << "You entered: " << myNumber << endl << endl;
   return myNumber;
}

void imageCapture(string command, int camNum, int camType, bool writeFile = false)
{
   if (camType == WEBCAM)
   {
      webcam_command_.command = command;
      webcam_command_.camera_number = camNum;
      if (writeFile) webcam_command_.write_file = true;
      else webcam_command_.write_file = false;
      webcam_pub_.publish(webcam_command_);
   }
   else if (camType == DIGCAM)
   {
      digcam_command_.command = command;
      digcam_command_.camera_number = camNum;
      if (writeFile) digcam_command_.write_file = true;
      else digcam_command_.write_file = false;
      digcam_pub_.publish(digcam_command_); 
   }
}     

void setZoom(int camNum, float zoom)
{
   digcam_command_.command = "setZoom";
   digcam_command_.camera_number = camNum;
   digcam_command_.zoom = zoom;
   digcam_pub_.publish(digcam_command_); 
}  

void testCameras()
{
   // the webcams:
   imageCapture("cap_and_rel", FRONT_CAMERA_NUMBER, WEBCAM);
   imageCapture("cap_and_rel", REAR_CAMERA_NUMBER, WEBCAM);     


   // the digital cams
   setZoom(FRONT_CAMERA_NUMBER, 9.);
   setZoom(REAR_CAMERA_NUMBER, 9.);
   imageCapture("capture", FRONT_CAMERA_NUMBER, DIGCAM);
   imageCapture("capture", REAR_CAMERA_NUMBER, DIGCAM);
}  

void homeCenterCallback(const outdoor_bot::NavTargets_msg::ConstPtr &msg)
{
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   range_ = msg->range;
   homeCenterUpdated_ = true;
   ROS_INFO("home updated in control: center, range = (%d, %d), %f", centerX_, centerY_, range_);
   cout << "home center received data" << endl;
}

void headForFirstTarget()
{
   // move off the platform and straight forward 10m
   // take photo
   // do we have the target in sight
      // if so, estimate range, if < 20m go to approach target routine
      // otherwise, center on it and then go out 10m and loop to take photo
   // if not, sweep camera to find target, if found loop to "we have the target in sight"
   // if not found, use dead reckoning to get to within about 20m of known location and loop to sweep camera
}

void approachTarget()
{
   // center target, estimate range
   // move in until target is < 1m
}

void captureTarget()
{
   // center target, estimate range
   // lower front loader
   // move forward to target contact with loader
   // decide if drop bar is needed, if so, drop it and move forward to engage it
   // lift loader, move shade to next bin slot
   // verify that target is in loader
   // if drop bar deployed, move in reverse and wind it back up
   // dump target into bin
   // move forward 10m or perhaps farther to find flat ground and to get out of target area range
   // verify that target is in bin
}

void searchForTarget()
{
   // sweep cameras
   // if target found, go to approach target
   // rotate 90 degrees and loop to sweep cameras
   // rotate 180 degrees and loop to sweep cameras
}   
   
bool headForHome()
{
   std_msgs::String msgMotion;
   char cmd[32];

   if (homeCenterUpdated_)
   {
      homeCenterUpdated_ = false;
      if (centerX_ < 0) return false;  // no home target was found
      // center the target and move forward
      double offsetX = ((double) (totalX_ - centerX_)) / ((double) totalX_);
      //double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
      if (fabs(offsetX) > 0.1)
      {
         if (range_ > 20.)
         {
            //move forward 10m and turn by the angle needed to center the target in X
            ROS_INFO("moving forward 10m and centering the home target");
         }
         else
         {
            // turn by the angle needed to center the target in X
            ROS_INFO("centering the home target");
         }
      }
      else
      {
         if (range_ > 15.)
         {
            //move forward 10m
            cmd[0] = 'n';
            cmd[1] = 0; // end with null character
            msgMotion.data = cmd;
            motion_pub_.publish(msgMotion);
            ROS_INFO("moving forward 10m");
         }
         else if (range_ > 10.)
         {
            //move forward 5 m
            ROS_INFO("moving forward 5m");
         }
         else if (range_ > 5.)
         {
            //move forward 3 m
            ROS_INFO("moving forward 5m");
         }
         else 
         {
            // final approach
            ROS_INFO("moving to home spot");
         }
      }
   }
   else
   {
      cout << "home center not updated since last call to headForHome" << endl;
      return false; // no home center update yet
   }
   return true;
}

bool CallNavTargetsService(string filename)
{
   outdoor_bot::NavTargets_service::Request req;
   outdoor_bot::NavTargets_service::Response resp;
   req.image_filename = filename;
   bool success = NavTargets_client_.call(req,resp);
   if (success)
   {
      centerX_ = resp.centerX;
      centerY_ = resp.centerY;
      range_ = resp.range;
      totalX_ = resp.totalX;
      homeCenterUpdated_ = true;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("NavTargets service call failed");
   return false; // false if service call failed
}

bool CallMainTargetsService(string filename, bool firstTarget = false)
{
   outdoor_bot::mainTargets_service::Request req;
   outdoor_bot::mainTargets_service::Response resp;
   req.image_filename = filename;
   req.firstTarget = firstTarget;
   req.approxRange = approxRangeToTarget_;
   bool success = mainTargets_client_.call(req,resp);
   
   if (success)
   {
      newMainTargetImageReceived_ = resp.newImageReceived;
      if ((!filename.compare("memory")) && (!newMainTargetImageReceived_)) return true;
      centerX_ = resp.centerX;
      centerY_ = resp.centerY;
      float rangeSquared = resp.rangeSquared;
      if (rangeSquared >= 0) range_ = sqrt(rangeSquared);
      else
      {
         range_ = 20;
         rangeUnknown_ = true;
      }
      totalX_ = resp.totalX;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("mainTargets service call failed");
   return false; // false if service call failed
}

bool CallDigcamsService(int camNum, string command, string filename, float zoom, bool writeFile = false)
{
   outdoor_bot::digcams_service::Request req;
   outdoor_bot::digcams_service::Response resp;
   req.filename = filename;
   req.command = command;
   req.camera_number = camNum;
   req.zoom = zoom;
   req.write_file = writeFile;
   bool success = digcams_client_.call(req,resp);
   if (success)
   {
      retCapToMemory_ = resp.captureResult;
      zoomResult_ = resp.zoomResult;
      writeFileResult_ = resp.writeFileResult;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("mainTargets service call failed");
   return false; // false if service call failed
}


int main(int argc, char* argv[])
{
   ros::init(argc, argv, "autonomous_node");
   ros::NodeHandle nh;

   digcams_client_ = nh.serviceClient<outdoor_bot::digcams_service>("digcamsService");
   NavTargets_client_ = nh.serviceClient<outdoor_bot::NavTargets_service>("NavTargetsService");
   mainTargets_client_ = nh.serviceClient<outdoor_bot::mainTargets_service>("mainTargetsService");
   webcam_pub_ = nh.advertise<outdoor_bot::webcams_custom>("webcam_cmd", 25);
   digcam_pub_ = nh.advertise<outdoor_bot::digcams_custom>("digcam_cmd", 25);
   home_center_ = nh.subscribe("Home_target_center", 10, homeCenterCallback); 

   approxRangeToTarget_ = 70.; // we start at about 70m from the first target
   range_ = 70;
   rangeUnknown_ = false;
   retCapToMemory_ = -1;
   zoomResult_ = false;
   writeFileResult_ = false;
   newMainTargetImageReceived_ = false;
   homeCenterUpdated_ = false;

   // first step is to see if we have a target to track
   // if so, we will track it and then try to pick it up
   // we will keep track of the number of targets we have and also the time
   // we will also listen for command inputs
   // if we have the correct number of targets and time is getting short
   // we will head for home and dock
   // all along we need to monitor for pause command

   // give some time for everyone to setup and get started
   getUserInputInteger();
   centerX_ = -1; 
   string filenameToWrite = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
   
/*
   // start by setting zooms and taking a photo from each camera
   //testCameras();

   // now we will see if we can find the first target with the digital camera full zoom

   if (CallDigcamsService(FRONT_CAMERA_NUMBER, "setZoom", filenameToWrite, 6., false)) cout << "zoom service done" << endl;
  
   // the digital cams
   ROS_INFO("digcams capture service sent now");
   if (CallDigcamsService(FRONT_CAMERA_NUMBER, "capture", filenameToWrite, 8., false)) cout << "capture service done" << endl;
   ROS_INFO("digcams capture service returned now");
   string filenameMem = "memory";
   sleep(1);   // give time for the image file to be read by mainTargets

     
   if (CallMainTargetsService(filenameMem, true))
   {
      for (int i=0; i < 5; i++)
      {
         if (newMainTargetImageReceived_)
         {
            ROS_INFO("mainTargets received a new image");
            break;
         }
         else
         {
            ROS_INFO("stuck waiting for image transport");
            ros::spinOnce();
            sleep(1);   // give time for the image file to be read by mainTargets
            CallMainTargetsService(filenameMem, true);
         }
      }
   }
   else cout << "mainTargets service call failed" << endl;

   */ 
   /*
   // see if we can find the first target
   string filename;
   for (int i = 1; i < 9; i++)
   {
      //if (i==0) filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
      //if (i==1) filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0_scaled.jpg";
      //if (i==1) filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_visible_60m_fullzoom.JPG";
      if (i==1)
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_70m.JPG";
         approxRangeToTarget_ = 70.;
      }
      if (i==2)
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/grass_with_white_lines_at_50m.JPG";
         approxRangeToTarget_ = 60.;
      }
      if (i==3)
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_50m.JPG";
         approxRangeToTarget_ = 50.;
      }
      if (i==4)
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_40m.JPG";
         approxRangeToTarget_ = 40.;
      }
      if (i==5) 
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_30m.JPG";
         approxRangeToTarget_ = 30.;
      }
      if (i==6) 
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_20m.JPG";
         approxRangeToTarget_ = 20.;
      }
      if (i==7) 
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white_10m.JPG";
         approxRangeToTarget_ = 10.;
      }
      if (i==8) 
      {
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/white1.JPG";
         approxRangeToTarget_ = 2.;
      }
      //if (i==2) filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0_slightly_cropped.jpg";
      //if (i==3) filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/200mm_crouching.JPG";
      //if (i==4) filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300_mm_crouching.JPG"; 
      //if (i==5) filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_crouching _with_tree_near_target.JPG";

      if (CallMainTargetsService(filename, true))
      {
         cout << " filename: " << filename << endl; 
            
         if (centerX_ > 0)
         {
            cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << range_ << endl;
         }
         else
         {
            cout << "no target found yet" << endl;
         }
         cout << endl;
      }
   }

   // if we found a target, we would now move the robot to center the target in the camera, take another image
   // and then move forward half the distance to the target until we are inside about 3 to 5 meters
   // then we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up

   // if we have not found a target, then we would either move closer or pan the camera and do another scan
   */


   // drive to target
   
   // look for any target
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/red_target_close.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/pink_target_very_close_small.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/pink_target_closer.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_pink.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/200mm_scan12.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_yellow_close.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_yellow.JPG";
   for (int i=0; i < 75; i++)
   {
      centerX_ = -1;
      std::stringstream buffer;
      buffer << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/webcam/webcam1_" << i << ".jpg";
      std::string filenm = buffer.str();
      CallMainTargetsService(filenm, false);
      cout << " filename: " << filenm << endl; 
      if (centerX_ > 0)
      {
         cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << range_ << endl;
      }
      else
      {
         cout << "no target found yet" << endl;
      }
      cout << endl;

   }
   /*
   // see if we can find home
   bool homeTargetFound = false;
   // first try the webcams
   imageCapture("cap_and_rel", FRONT_CAMERA_NUMBER, WEBCAM, true);
   getUserInputInteger();
   filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam0.jpg";
   if (CallNavTargetsService(filename))
   {
      if (!headForHome())
      {
         imageCapture("cap_and_rel", REAR_CAMERA_NUMBER, WEBCAM, true);
         getUserInputInteger();
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam1.jpg";
         if (CallNavTargetsService(filename))
         {
            if (!headForHome())
            
            {            
               homeCenterUpdated_ = false;
               imageCapture("capture", FRONT_CAMERA_NUMBER, DIGCAM, true);
               getUserInputInteger();
               string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
               if (CallNavTargetsService(filename))               
               {
                  if (!headForHome())
                  {
                     cout << "Still looking for home, centerX_ = " << centerX_ << endl;
                     homeCenterUpdated_ = false;
                     imageCapture("capture", REAR_CAMERA_NUMBER, DIGCAM, true);
                     getUserInputInteger();
                     string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
                     if (CallNavTargetsService(filename))
                     {   
                        if (!headForHome())
                        {
                           cout << "Still looking for home" << endl;
                           // at this point, we should use the radar ranging and dir antenna to move closer   
                        } // fourth ! head for home
                        else homeTargetFound = true;
                      } // fourth service call
                   }    // third ! head for home
                   else homeTargetFound = true;
               }        // third service call.               
            }           // second ! head for home
            else homeTargetFound = true;
         }              // second service call
      }                 // first ! head for home
      else homeTargetFound = true;
   }                    // first service call
   if (homeTargetFound)
   {
      cout << "Heading for home" << endl;
      // steer to image center and roll toward home
   }
   */
   ros::spin();
   return EXIT_SUCCESS;
}

