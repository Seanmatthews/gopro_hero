#ifndef GOPRO_HERO_NODE_HPP_
#define GOPRO_HERO_NODE_HPP_

#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

#include <boost/thread/thread.hpp>

#include "gopro_hero/gopro_hero.hpp"
#include "gopro_hero_msgs/Shutter.h"
#include "gopro_hero_msgs/SettingsMap.h"


namespace rowboat1
{
    class GoProHeroNode
    {
    public:
        GoProHeroNode(ros::NodeHandle nh);
        ~GoProHeroNode();

        void init();
        
    private:
        void mainLoop();

        void modeCB(const std_msgs::Int8::ConstPtr& msg);
        void cameraSettingsCB(const gopro_hero_msgs::SettingsMap::ConstPtr& msg);
        bool triggerShutterCB(gopro_hero_msgs::Shutter::Request& req,
                              gopro_hero_msgs::Shutter::Response& rsp);
        void toggleVideoStreamCB(const std_msgs::Bool::ConstPtr& msg);
        
        static void processStreamFrameCB(int width, int height, int numBytes, uint8_t* bytes);
        
        ros::NodeHandle nh_;
        ros::Publisher imageStreamPub_;
        ros::Subscriber toggleVideoStream_;
        ros::Subscriber modeSub_;
        ros::Subscriber cameraSettingsSub_;
        ros::ServiceServer shutterTriggerSrv_;
        
        GoProHero gp_;
        bool isStreaming_;
        boost::thread streamThread_;
        
    };
}

#endif
