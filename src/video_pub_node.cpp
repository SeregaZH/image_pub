
#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <dirent.h>

// The contant is substitute of enum value CV_CAP_PROP_POS_MSEC
// The value is not compatible with different version of OPEN CV
// We temporary hardcode the constant. Later it should be replaced with more relaible code to support 
// different opencv versions
#define CV_CAP_PROP_POS_MSEC_SUBSTITUTE 0

bool endsWith(const std::string& s, const std::string& suffix)
{
    return (int)s.rfind(suffix) == std::abs((int)(s.size()-suffix.size()));
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting video_pub ROS node...\n");

    ros::init(argc, argv, "video_pub");
    ros::NodeHandle nh("~");

    std::string camera_topic;
    std::string camera_info_topic;
    std::string camera_info_url;
    std::string video_path;
    std::string frame_id;
    float       pub_rate;
    int         start_sec;
    bool        repeat;
    nh.param<std::string>("camera_topic",      camera_topic,      "/camera/image_raw");
    nh.param<std::string>("camera_info_topic", camera_info_topic, "/camera/camera_info");
    nh.param<std::string>("camera_info_url",   camera_info_url,   "");
    nh.param<std::string>("video_path", video_path, "");
    nh.param<std::string>("frame_id", frame_id, "");
    nh.param("pub_rate",  pub_rate, 30.0f);
    nh.param("start_sec", start_sec, 0);
    nh.param("repeat",    repeat, true);

    ROS_INFO("CTopic : %s", camera_topic.c_str());
    ROS_INFO("ITopic : %s", camera_info_topic.c_str());
    ROS_INFO("CI URL : %s", camera_info_url.c_str());
    ROS_INFO("Source : %s", video_path.c_str());
    ROS_INFO("Rate   : %.1f", pub_rate);
    ROS_INFO("Start  : %d", start_sec);
    ROS_INFO("Repeat : %s", repeat ? "yes" : "no");
    ROS_INFO("FrameID: %s", frame_id.c_str());
    try
    {
        camera_info_manager::CameraInfoManager camera_info_manager(nh);
        if (camera_info_manager.validateURL(camera_info_url))
            ROS_INFO("Valid camera_info_url: %s", camera_info_url.c_str());
            camera_info_manager.loadCameraInfo(camera_info_url);

        ros::Publisher img_pub  = nh.advertise<sensor_msgs::Image>(camera_topic, 1);
        ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);

        ros::Rate rate(pub_rate);

        char* file_path = (char*)malloc(256);

        while (ros::ok())
        {
            DIR* dirp = opendir(video_path.c_str());
            
            if (!dirp) 
            {
                ROS_ERROR("Unable to open directory: %s", video_path.c_str());
            }
            
            struct dirent * dp;
            bool isVideoExist = false;

            while((dp = readdir(dirp)) != NULL && ros::ok()) 
            {
                if (!endsWith(dp->d_name, ".mp4"))
                {
                    continue;
                }

                // At least one video was founded
                isVideoExist = true;
                
                stpcpy(file_path, (char*)video_path.c_str());
                file_path = strcat(strcat(file_path, "/"), dp->d_name);

                ROS_INFO("Video file %s has been read", file_path);
                cv::VideoCapture vid_cap(file_path);
                if (start_sec > 0)
                    vid_cap.set(CV_CAP_PROP_POS_MSEC_SUBSTITUTE, 1000.0 * start_sec);

                if (!vid_cap.isOpened())
                {
                    ROS_ERROR("Cannot read video. Try moving video file to sample directory.");
                    return -1;
                }
                

                while (ros::ok()) 
                {
                    cv::Mat img;
                    bool image_captured = vid_cap.read(img);
                    if (!image_captured)
                    {
                        break;
                    }
                    else
                    {
                        //ROS_DEBUG("Image: %dx%dx%d, %zu, %d", img.rows, img.cols, img.channels(), img.elemSize(), img.type() == CV_8UC3);
                        if (img.type() != CV_8UC3)
                            img.convertTo(img, CV_8UC3);
                        // Convert image from BGR format used by OpenCV to RGB.
                        #if (CV_VERSION_MAJOR >= 4) 
                            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
                        #else
                            cv::cvtColor(img, img, CV_BGR2RGB); 
                        #endif

                        auto img_msg = boost::make_shared<sensor_msgs::Image>();
                        img_msg->header.stamp    = ros::Time::now();
                        img_msg->header.frame_id = frame_id;
                        img_msg->encoding = "rgb8";
                        img_msg->width = img.cols;
                        img_msg->height = img.rows;
                        img_msg->step = img_msg->width * img.channels();
                        auto ptr = img.ptr<unsigned char>(0);
                        img_msg->data = std::vector<unsigned char>(ptr, ptr + img_msg->step * img_msg->height);
                        img_pub.publish(img_msg);

                        if (camera_info_manager.isCalibrated())
                        {
                            auto info = boost::make_shared<sensor_msgs::CameraInfo>(camera_info_manager.getCameraInfo());
                            info->header = img_msg->header;
                            info_pub.publish(info);
                        }
                        else
                        {
                            ROS_DEBUG("Camera not calibrated");
                        }
                    }
                    //ROS_DEBUG("Image: %d", image_captured);
                    ros::spinOnce();
                    rate.sleep();
                }

            }

            closedir(dirp);
            if(!repeat || !isVideoExist)
            {
                ROS_INFO("The video directory was read to the end. Check that repeat option are specified or video files exist in the folder");
                return 0;
            }
        }
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR("Error occured: %s ", ex.what());
    }
    catch (...) 
    {
        ROS_ERROR("Error occured");
    }

    return 0;
}