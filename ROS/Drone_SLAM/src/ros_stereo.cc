/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<nav_msgs/Odometry.h> 
#include<geometry_msgs/Point.h> 
#include<pangolin/pangolin.h>
#include<std_msgs/UInt8.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h" //please make sure the path if errors occur

using namespace std;

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4) {
        cerr << endl << "Usage: rosrun Drone_SLAM Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    cout<< "Welcome to use Drone_SLAM!"<< endl;
    
    bool bLoadMap = false;
    string type_in;
    while (1) {
        cout << "Do you want to load map?(y/n)" << endl;
        cin >> type_in;
        if(type_in == "Y" || type_in == "y" || type_in == "yes" || type_in == "Yes"){
            bLoadMap = true;
            break;
        } 
        else if (type_in == "N" || type_in == "n" || type_in == "No" || type_in == "no") {
            break;
        } 
        else {
            cout << "Wrong input. Please input again.";
        }
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // fifth parameter to set save map true
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true, bLoadMap);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify) {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened()) {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }
        // calculate stereo rectify matrix
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    //get data from ros node
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed/zed_node/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed/zed_node/right/image_rect_color", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    bool bSaveMap = false;
    type_in.clear();
    while (1) {
        cout << endl << "Do you want to save the map?(y/n)" << endl;
        cin >> type_in;
        if(type_in == "Y" || type_in == "y" || type_in == "yes" || type_in == "Yes"){
            bSaveMap = true;
            break;
        } 
        else if (type_in == "N" || type_in == "n" || type_in == "No" || type_in == "no") {
            break;
        } 
        else {
            cout << "Wrong input. Please input again.";
        }
    }

    if (bSaveMap == true) {
        cout << "Input the name of the MAP" << endl
             << "the MAP will be saved as a binary file in your current working directory. "
             << "ex. If your input is zedmap, a MAP file named zedmap.bin will show up." << endl;
        cout << "Input the MAP's name: ";
        string name_of_map;
        cin >> name_of_map;
        if (!hasEnding(name_of_map, ".bin")) {
            name_of_map += ".bin";
        }
        SLAM.SaveMap(name_of_map);
    }

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify) {
        // use the rectify matrix to rectyify two images from left and right
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    } 
    else {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
}
