#include <ros/ros.h>
#include "outdoor_bot/webcams_custom.h"
#include "outdoor_bot/digcams_service.h"
#include "outdoor_bot/digcams_custom.h"
#include "outdoor_bot/NavTargets_service.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/mainTargets_msg.h"
#include "outdoor_bot/radar_service.h"
#include "outdoor_bot/setPose_service.h"
#include "FBFSM/FBFSM.h"
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
#define NUM_RADARS 3
#define plat1_X 2269.5
#define plat1_Y 832.5
#define plat1_Yaw 0.
#define plat2_X 2203.5
#define plat2_Y 915.
#define plat2_Yaw 0.
#define plat3_X 2143.5
#define plat3_Y 990.
#define plat3_Yaw 0.
#define target1_X 2505.
#define target1_Y 1504.
#define PIXELS_PER_METER 10.6
#define METERS_PER_PIXEL 0.0943

ros::ServiceClient NavTargets_client_, mainTargets_client_, digcams_client_, radar_client_, setPose_client_;
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
double distanceToHome_[NUM_RADARS], deltaDistanceToHome_[NUM_RADARS], velocityFromHome_[NUM_RADARS];
int platformXPose_[3], platPoseX_;
int platformYPose_[3], platPoseY_;
int platformYawPose_[3], platPoseYaw_;
int targetXPose_, targetYPose_;
bool phase1_, pauseCommanded_, previousState_;

FBFSM fsm_;
int BootupState_, FindFirstTargetState_, MoveToFirstTargetState_, PickupFirstTargetState_;
int FindTargetState_, MoveToTargetState_, PickupTargetState_;
int FindHomeState_, HeadForHomeState_, MoveOntoPlatformState_;
int PauseState_;
int platformNumber_;

bool getUserInput()
{
   string input = "";
   cout << "Please enter the platform number we are starting on: " << endl << endl;
   getline(cin, input);

   // This code converts from string to number safely.
   stringstream myStream(input);
   if (myStream >> platformNumber_)
   {
      if (platformNumber_ == 1 || platformNumber_ == 2 || platformNumber_ == 3) 
         cout << "We are one platform number " << platformNumber_ << endl << endl;
      else
      {
         cout << "Invalid platform number, please try again" << endl;
         return false;
      }
   }
   else
   {
      cout << "Failed to get an input platform number" << endl;
      return false;
   }
   

   phase1_ = true;
   int compNumber;
   cout << "Please enter the competition phase (1 or 2): " << endl << endl;
   getline(cin, input);
   stringstream myStream1(input);
   if (myStream1 >> compNumber)
   {
      if (compNumber == 1) phase1_ = true;
      else if (compNumber == 2) phase1_ = false;   // we are in phase 2
      else
      {
         cout << "Invalid competition phase number, please try again" << endl;
         return false;
      }
      cout << "We are in competition phase " << compNumber << endl;
      return true;
   }
   cout << "Failed to get an input competition phase number" << endl;
   return false;
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

bool callRadarService(int radarNumber = 0)
{
   if (radarNumber < 0 || radarNumber > NUM_RADARS - 1) return false;
   outdoor_bot::radar_service::Request req;
   outdoor_bot::radar_service::Response resp;
   req.radarNumber = radarNumber;
   bool success = radar_client_.call(req,resp);
   if (success)
   { 
      if (resp.distanceToHome > 0.)
      {
         distanceToHome_[radarNumber] = resp.distanceToHome;
         deltaDistanceToHome_[radarNumber] = resp.deltaDistanceToHome;
         velocityFromHome_[radarNumber] = resp.velocityFromHome;
      }
      return true;
   }
   return false;
}

bool callSetPoseService(double x, double y, double yaw, bool setHome)
{
   outdoor_bot::setPose_service::Request req;
   outdoor_bot::setPose_service::Response resp;
   req.setHome = setHome;
   req.x = x;
   req.y = y;
   req.yaw = yaw; 
   if (setPose_client_.call(req,resp)) return true;
   return false;
}

void on_enter_BootupState()
{
   approxRangeToTarget_ = 70.; // we start at about 70m from the first target
   range_ = 70;
   rangeUnknown_ = false;
   retCapToMemory_ = -1;
   zoomResult_ = false;
   writeFileResult_ = false;
   newMainTargetImageReceived_ = false;
   homeCenterUpdated_ = false;
   for (int i=0; i < NUM_RADARS; i++)
   {
      distanceToHome_[i] = 0.;
      deltaDistanceToHome_[i] = 0.;
      velocityFromHome_[i] = 0.;
   }

   platformXPose_[0] = plat1_X * METERS_PER_PIXEL;
   platformXPose_[1] = plat2_X * METERS_PER_PIXEL;
   platformXPose_[2] = plat3_X * METERS_PER_PIXEL;
   platformYPose_[0] = plat1_Y * METERS_PER_PIXEL;
   platformYPose_[1] = plat2_Y * METERS_PER_PIXEL;
   platformYPose_[2] = plat3_Y * METERS_PER_PIXEL;
   platformYawPose_[0] = plat1_Yaw;
   platformYawPose_[1] = plat2_Yaw;
   platformYawPose_[2] = plat3_Yaw;
   targetXPose_ = target1_X * METERS_PER_PIXEL;
   targetYPose_ = target1_Y * METERS_PER_PIXEL;
}

int on_update_BootupState()
{

   // give some time for everyone to setup and get started, then ask what platform we are assigned
   if (!getUserInput()) return BootupState_;
   if (platformNumber_ >= 1 && platformNumber_ < 4)
   {
      platPoseX_ = platformXPose_[platformNumber_ - 1];
      platPoseY_ = platformYPose_[platformNumber_ - 1];
      platPoseYaw_ = platformYawPose_[platformNumber_ - 1];
   }
   else
   {
      cout << "invalid platform number selected, you selected " << platformNumber_ << endl;
      cout << "start again" << endl;
      return BootupState_;
   }
   if (!callSetPoseService(platPoseX_, platPoseY_, platPoseYaw_, true)) // set home pose in robotPose_node
   {
      cout << "failed to set home pose, need to start again" << endl;
      return BootupState_;
   }
   if (!callSetPoseService(platPoseX_, platPoseY_, platPoseYaw_, false)) // and place us there
   {
      cout << "failed to place bot at home, need to start again" << endl;
      return BootupState_;
   }

   centerX_ = -1; 

   // get radar ranges
   if (callRadarService(0))
   {
      if (distanceToHome_[0] > 0)
      {
         cout << "radar data: " << distanceToHome_[0] << ", " << deltaDistanceToHome_[0] << ", " << velocityFromHome_[0] << endl; 
      }
      else cout << "radar data not obtained" << endl;
   }  
   else
   {
      cout << "radar service failed" << endl;
      return BootupState_;
   }
   // start by setting zooms and taking a photo from each camera
   testCameras();
   previousState_ = BootupState_;
   return FindFirstTargetState_;
}

int on_update_FindFirstTargetState()
{
   string filenameToWrite = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
   // see if we can find the first target with the digital camera full zoom
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
   else
   {
      cout << "mainTargets service call failed" << endl;
      return FindFirstTargetState_;
   }

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
   if (centerX_ > 0) return MoveToFirstTargetState_;
   return FindFirstTargetState_;
}

int on_update_MoveToFirstTargetState()
{
   // if we don't see the target yet, move off the platform and straight forward 10m
   // take photo
   // do we have the target in sight
      // if so, estimate range, if < 3 to 5m go to approach target routine
      // otherwise, center on it and then go out 10m and loop to take photo
   // if not, sweep camera to find target, if found loop to "we have the target in sight"
   // if not found, use dead reckoning to get to within about 20m of known location and loop to sweep camera
   return PickupFirstTargetState_;
}

int on_update_PickupFirstTargetState()
{
   // here we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up
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
   if (phase1_) return FindHomeState_;
   return FindTargetState_;
}
 
int on_update_FindTargetState()
{
   // sweep cameras
   // if target found, go to approach target
   // rotate 90 degrees and loop to sweep cameras
   // rotate 180 degrees and loop to sweep cameras

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
   if (centerX_ > 0) return MoveToTargetState_;
   return FindTargetState_;
}

int on_update_MoveToTargetState()
{
   // we found a target, we now move the robot to center the target in the camera, take another image
   // and then move forward half the distance to the target until we are inside about 3 to 5 meters   
   return PickupTargetState_;
}

int on_update_PickupTargetState()
{
   // here we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up
   return FindTargetState_;
}

int on_update_HeadForHomeState()
{
   std_msgs::String msgMotion;
   char cmd[32];

   if (homeCenterUpdated_)
   {
      homeCenterUpdated_ = false;
      if (centerX_ < 0) return FindHomeState_;  // no home target was found
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
      return FindHomeState_; // no home center update yet
   }
   return MoveOntoPlatformState_;
}

int on_update_FindHomeState()
{
   // see if we can find home
   bool homeTargetFound = false;
   // first try the webcams
   imageCapture("cap_and_rel", FRONT_CAMERA_NUMBER, WEBCAM, true);
//   getUserInput();
   string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam0.jpg";
   if (CallNavTargetsService(filename)) return HeadForHomeState_;

   /*if (!enterHeadForHomeState())
      {
         imageCapture("cap_and_rel", REAR_CAMERA_NUMBER, WEBCAM, true);
         getUserInputInteger();
         filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam1.jpg";
         if (CallNavTargetsService(filename))
         {
            if (!enterHeadForHomeState())
            
            {            
               homeCenterUpdated_ = false;
               imageCapture("capture", FRONT_CAMERA_NUMBER, DIGCAM, true);
               getUserInputInteger();
               string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
               if (CallNavTargetsService(filename))               
               {
                  if (!enterHeadForHomeState())
                  {
                     cout << "Still looking for home, centerX_ = " << centerX_ << endl;
                     homeCenterUpdated_ = false;
                     imageCapture("capture", REAR_CAMERA_NUMBER, DIGCAM, true);
                     getUserInputInteger();
                     string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
                     if (CallNavTargetsService(filename))
                     {   
                        if (!enterHeadForHomeState())
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
      return MoveToHomeState_;
   }
   */
   return FindHomeState_;
}

int on_update_MoveOntoPlatformState()
{
   // drive onto the platform and then pause
   return PauseState_;
}

void on_enter_PauseState()
{
   // remember variables and other things needed to recover when motion is resumed
}
   
int on_update_PauseState()
{
   if (pauseCommanded_) return PauseState_;
   return previousState_;
}

void on_exit_PauseState()
{
   // we need to recover from the pause before going back to the previous state
}

void setupStates()
{
   BootupState_ = fsm_.add_state();  // 0
   FindFirstTargetState_ = fsm_.add_state();  // 1
   MoveToFirstTargetState_ = fsm_.add_state();
   PickupFirstTargetState_ = fsm_.add_state();
   FindHomeState_ = fsm_.add_state();
   HeadForHomeState_ = fsm_.add_state();
   MoveOntoPlatformState_ = fsm_.add_state();
   PauseState_ = fsm_.add_state();

   fsm_.set_entry_function(BootupState_, boost::bind(&on_enter_BootupState));
   fsm_.set_update_function(BootupState_, boost::bind(&on_update_BootupState));
   //fsm_.set_exit_function(BootupState_, boost::bind(&on_exit_BootupState));

   //fsm_.set_entry_function(FindFirstTargetState_, boost::bind(&on_enter_FindFirstTargetState));
   fsm_.set_update_function(FindFirstTargetState_, boost::bind(&on_update_FindFirstTargetState));
   //fsm_.set_exit_function(FindFirstTargetState_, boost::bind(&on_exit_FindFirstTargetState));

   //fsm_.set_entry_function(MoveToFirstTargetState_, boost::bind(&on_enter_MoveToFirstTargetState));
   fsm_.set_update_function(MoveToFirstTargetState_, boost::bind(&on_update_MoveToFirstTargetState));
   //fsm_.set_exit_function(MoveToFirstTargetState_, boost::bind(&on_exit_MoveToFirstTargetState));

   //fsm_.set_entry_function(PickupFirstTargetState_, boost::bind(&on_enter_PickupFirstTargetState));
   fsm_.set_update_function(PickupFirstTargetState_, boost::bind(&on_update_PickupFirstTargetState));
   //fsm_.set_exit_function(PickupFirstTargetState_, boost::bind(&on_exit_PickupFirstTargetState));

   //fsm_.set_entry_function(FindTargetState_, boost::bind(&on_enter_FindTargetState));
   fsm_.set_update_function(FindTargetState_, boost::bind(&on_update_FindTargetState));
   //fsm_.set_exit_function(FindTargetState_, boost::bind(&on_exit_FindTargetState));

   //fsm_.set_entry_function(PickupTargetState_, boost::bind(&on_enter_PickupTargetState));
   fsm_.set_update_function(PickupTargetState_, boost::bind(&on_update_PickupTargetState));
   //fsm_.set_exit_function(PickupTargetState_, boost::bind(&on_exit_PickupTargetState));

   //fsm_.set_entry_function(FindHomeState_, boost::bind(&on_enter_FindHomeState));
   fsm_.set_update_function(FindHomeState_, boost::bind(&on_update_FindHomeState));
   //fsm_.set_exit_function(FindHomeState_, boost::bind(&on_exit_FindHomeState));

   //fsm_.set_entry_function(HeadForHomeState_, boost::bind(&on_enter_HeadForHomeState));
   fsm_.set_update_function(HeadForHomeState_, boost::bind(&on_update_HeadForHomeState));
   //fsm_.set_exit_function(HeadForHomeState_, boost::bind(&on_exit_HeadForHomeState));

   //fsm_.set_entry_function(MoveOntoPlatformState_, boost::bind(&on_enter_MoveOntoPlatformState));
   fsm_.set_update_function(MoveOntoPlatformState_, boost::bind(&on_update_MoveOntoPlatformState));
   //fsm_.set_exit_function(MoveOntoPlatformState_, boost::bind(&on_exit_MoveOntoPlatformState));

   fsm_.set_entry_function(PauseState_, boost::bind(&on_enter_PauseState));
   fsm_.set_update_function(PauseState_, boost::bind(&on_update_PauseState));
   fsm_.set_exit_function(PauseState_, boost::bind(&on_exit_PauseState));

}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "autonomous_node");
   ros::NodeHandle nh;

   digcams_client_ = nh.serviceClient<outdoor_bot::digcams_service>("digcams_service");
   radar_client_ = nh.serviceClient<outdoor_bot::radar_service>("radar_service");
   setPose_client_ = nh.serviceClient<outdoor_bot::setPose_service>("setPose_service");
   NavTargets_client_ = nh.serviceClient<outdoor_bot::NavTargets_service>("NavTargetsService");
   mainTargets_client_ = nh.serviceClient<outdoor_bot::mainTargets_service>("mainTargetsService");
   webcam_pub_ = nh.advertise<outdoor_bot::webcams_custom>("webcam_cmd", 25);
   digcam_pub_ = nh.advertise<outdoor_bot::digcams_custom>("digcam_cmd", 25);
   home_center_ = nh.subscribe("Home_target_center", 10, homeCenterCallback); 

   pauseCommanded_ = true; // start in pause mode

   setupStates();

   // first step is to see if we have a target to track
   // if so, we will track it and then try to pick it up
   // we will keep track of the number of targets we have and also the time
   // we will also listen for command inputs
   // if we have the correct number of targets and time is getting short
   // we will head for home and dock
   // all along we need to monitor for pause command


   // we are ready to enter bootup state   
   previousState_ = BootupState_;
   fsm_.set_state(BootupState_);
   // and update....
   fsm_.update();
   
   // now we keep updating and nicely move through the states
   ros::spin();
   return EXIT_SUCCESS;
}

