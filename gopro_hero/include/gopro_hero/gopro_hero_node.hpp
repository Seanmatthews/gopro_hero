#ifndef GOPRO_HERO_NODE_HPP_
#define GOPRO_HERO_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include "gopro_hero/gopro_hero.hpp"
#include "gopro_hero/gopro_hero_stream.hpp"
#include "gopro_hero_msgs/Shutter.h"
#include "gopro_hero_msgs/SettingsMap.h"


namespace gopro_hero
{
    class GoProHeroNode
    {
    public:
        GoProHeroNode(ros::NodeHandle nh);
        ~GoProHeroNode();

        void init();
        
    private:
        void cameraSettingsCB(const gopro_hero_msgs::SettingsMap::ConstPtr& msg);
        bool triggerShutterCB(gopro_hero_msgs::Shutter::Request& req,
                              gopro_hero_msgs::Shutter::Response& rsp);
        void toggleVideoStreamCB(const std_msgs::Bool::ConstPtr& msg);
        void processStreamFrameCB(cv::Mat& frame);
        void streamErrorCB(std::string error);

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher imageStreamPub_;
        ros::Subscriber toggleVideoStream_;
        ros::Subscriber cameraSettingsSub_;
        ros::ServiceServer shutterTriggerSrv_;
        
        GoProHero gp_;
        GoProHeroStream *gpStream_;
        bool isStreaming_;
        
        const std::string host_ = "10.5.5.9";
        const unsigned int port_ = 8554;
    };
}

#endif
