#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include "../../../include/System.h"
#include "../../../include/Converter.h"
#include "../../../include/Utils.hpp"
#include <ctime>

#include <Camera.hpp>


using namespace std;
using namespace sl;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
    std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
    (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

void PublishPose(ros::Publisher* pub, const cv::Mat& Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        /*
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                            Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                            Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
            tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

            tf::Transform tfTcw(M,V);

            //mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
        */
        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(2);
        poseMSG.pose.position.z = twc.at<float>(1);

        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];

        poseMSG.header.frame_id = "VSLAM";
        poseMSG.header.stamp = ros::Time::now();
        cout << "PublishPose position x,y,z = \n" << poseMSG.pose.position << endl;

        pub->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
}

cv::Mat slMat2cvMat(sl::Mat& input);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tx2");
	cout<<"-----------1------------"<<endl;
    //ros::start();
    ros::NodeHandle nh_;
    ros::Publisher pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);
	cout<<"-----------2------------"<<endl;
    if(argc != 4) {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2_GPU tx2 [path to vocabulary] [path to settings] [whether use Map]" << endl;
        ros::shutdown();
        return 1;
    }
//------------------time--------------------------------------------
    time_t now = time(0);// 基于当前系统的当前日期/时间  
    tm *ltm = localtime(&now);
    char save_time[150]; 
    sprintf(save_time, "%d%d%d_%d_%d",1900 + ltm->tm_year,1 + ltm->tm_mon,ltm->tm_mday,ltm->tm_hour,ltm->tm_min);
    string save_time_name(save_time); 
//---------------------------------------------------------------------

//------------------setting------------------------------//

	cv::FileStorage fSettings("/home/nvidia/orb/ORB_SLAM2_GPU-master/gpu/TX2_ZED.yaml", cv::FileStorage::READ);


    int exposure = fSettings["Camera.exposure"];
	int contast  = fSettings["Camera.contrast"];
	int birght   = fSettings["Camera.bright"];
	int gain	 = fSettings["Camera.gain"];



//------------------setting------------------------------//


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool bReuseMap = false, bUseViewer = true;
    string MapName = "/home/nvidia/orb/ORB_SLAM2_GPU-master/Examples/ROS/ORB_SLAM2_GPU/map/"+save_time_name+".bin";
    if(strcmp(argv[3], "false") != 0)
    {
        bReuseMap = true;
        MapName = argv[3];
    }
	cout<<"-----------3------------"<<endl;
    ORB_SLAM2::System SLAM(
        argv[1], argv[2], ORB_SLAM2::System::STEREO, bUseViewer, bReuseMap, MapName);


    cout << "Start processing sequence ..." << endl;
   
//---------------------ZED init------------------------------------------------
   // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_NONE;
    //init_params.coordinate_units = UNIT_METER;
    init_params.camera_fps=fSettings["Camera.fps"];;




//-----------------------------------------------------------------



    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", errorCode2str(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }
//-----SDK URL:file:///usr/local/zed/doc/API/html/group__Enumerations.html#gafdecf198c85568238dacc827f5d085de
	zed.setCameraSettings(CAMERA_SETTINGS_BRIGHTNESS,birght);//Range:0~8
    zed.setCameraSettings(CAMERA_SETTINGS_CONTRAST,contast);//Range:0~8
    zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE,exposure);//Auto EXposure:-1 Manual Control Range:0~100
	zed.setCameraSettings(CAMERA_SETTINGS_GAIN,gain);		// gain 0-100
	
    cout<<"BRIGHTNESS:\t"<< zed.getCameraSettings(CAMERA_SETTINGS_BRIGHTNESS) <<endl;
    cout<<"CONTRAST:\t"<< zed.getCameraSettings(CAMERA_SETTINGS_CONTRAST) <<endl;
    cout<<"EXPOSURE:\t"<< zed.getCameraSettings(CAMERA_SETTINGS_EXPOSURE) <<endl;
	cout<<"GAIN:\t"<<zed.getCameraSettings(CAMERA_SETTINGS_GAIN)<<endl;


    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed_left(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_left = slMat2cvMat(image_zed_left);

    sl::Mat image_zed_right(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_right = slMat2cvMat(image_zed_right);

    // Main loop
    double trackTimeSum = 0.0;
    cv::Mat im;
    SET_CLOCK(t0);
    int frameNumber = 0;

    ros::Rate r(10);
    while (nh_.ok()) {
        
        if (zed.grab(runtime_parameters) == SUCCESS) 
		{
        	SET_CLOCK(t1);
       		double tframe = TIME_DIFF(t1, t0);

        	PUSH_RANGE("Track image", 4);
        	// Pass the image to the SLAM system
        	zed.retrieveImage(image_zed_left, VIEW_LEFT, MEM_CPU, new_width, new_height);
	    	zed.retrieveImage(image_zed_right, VIEW_RIGHT, MEM_CPU, new_width, new_height);

        	image_left = slMat2cvMat(image_zed_left);
        	image_right = slMat2cvMat(image_zed_right);

        	//cvtColor(image_left, image_left, CV_BGRA2BGR);
       		// cvtColor(Depth, frame_depth, CV_BGRA2BGR);
        	//cvtColor(image_right, image_right, CV_BGRA2BGR);

        	 cv::Mat Tcw = SLAM.TrackStereo(image_left,image_right, tframe);
			
 			PublishPose(&pos_pub, Tcw);
            POP_RANGE;

			SET_CLOCK(t2);

        	double trackTime = TIME_DIFF(t2, t1);
     		trackTimeSum += trackTime;
        	cout << "Frame Number" << frameNumber << " : Frame Time: " << tframe<< " Track Time: " << trackTime << endl;
            ++frameNumber;
        }
        
        
       
        ros::spinOnce();
        r.sleep();
    }
    // Stop all threads
    zed.close();
    SLAM.Shutdown();
    SLAM.SaveMap(MapName);

    cout << "Mean track time: " << trackTimeSum / frameNumber <<"\n";
    ros::shutdown();

    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}


