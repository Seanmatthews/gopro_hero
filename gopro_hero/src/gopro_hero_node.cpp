#include "gopro_hero/gopro_hero_node.hpp"
#include "sensor_msgs/CompressedImage.h"

using namespace std;
using namespace ros;

/**
 * The node does not have many primary functions-- triggering video & photo,
 * streaming video, and adjusting camera settings. However, there exist a
 * large number of settings. While the interal gopro_hero lib accesses these
 * settings individually, allowing such access through the ROS interface
 * would be cumbersome to develop and messy in a system of ROS nodes with
 * various params. To compartmentalize the camera params, users may set them
 * in groups of "camera", "video", "photo", and "multishot", using a
 * specialized hashmap-like message type.
 */

namespace rowboat1
{

    GoProHeroNode::GoProHeroNode(NodeHandle nh) :
        nh_(nh),
        isStreaming_(false)
    {

    }

    GoProHeroNode::~GoProHeroNode()
    {
        if (isStreaming_) streamThread_.interrupt();
    }

    
    void GoProHeroNode::init()
    {
        modeSub_ = nh_.subscribe("mode", 1, &GoProHeroNode::modeCB, this);
        imageStreamPub_ = nh_.advertise<sensor_msgs::CompressedImage>("stream", 5);
        toggleVideoStream_ = nh_.subscribe("toggle_video_stream", 1, &GoProHeroNode::toggleVideoStreamCB, this);
        cameraSettingsSub_ = nh_.subscribe("camera_settings", 1, &GoProHeroNode::cameraSettingsCB, this);
        shutterTriggerSrv_ = nh_.advertiseService("trigger_shutter", &GoProHeroNode::triggerShutterCB, this);
    }

    
    // Callback for setting the GoPro's mode
    void GoProHeroNode::modeCB(const std_msgs::Int8::ConstPtr& msg)
    {
        gp_.setMode(static_cast<GoProHero::Mode>(msg->data));
    }


    void GoProHeroNode::toggleVideoStreamCB(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data != isStreaming_)
        {
            isStreaming_ = !isStreaming_;
            if (msg->data) streamThread_ = boost::thread(&GoProHero::streamThreadFunc,
                                                         &GoProHeroNode::processStreamFrameCB);
            else streamThread_.interrupt();
        }
    }
    

    // NOTE no checks for proper enum value-- cast will occur regardless
    void GoProHeroNode::cameraSettingsCB(const gopro_hero_msgs::SettingsMap::ConstPtr& msg)
    {
        for (auto s : msg->settings)
        {
            auto val = s.id;
            auto name = s.name;
            
            if ("shutter" == name) gp_.shutter(val);
            else if ("orientation" == name) gp_.orientation(static_cast<Orientation>(val));
            else if ("ledBlink" == name) gp_.ledBlink(static_cast<LEDBlink>(val));
            else if ("beepVolume" == name) gp_.beepVolume(static_cast<BeepVolume>(val));
            else if ("lcdDisplay" == name) gp_.lcdDisplay(val);
            else if ("onScreenDisplay" == name) gp_.onScreenDisplay(val); 
            else if ("lcdBrightness" == name) gp_.lcdBrightness(static_cast<LCDBrightness>(val));
            else if ("lcdLock" == name) gp_.lcdLock(val);
            else if ("lcdSleepTimeout" == name) gp_.lcdSleepTimeout(static_cast<LCDSleepTimeout>(val));
            else if ( "autoOffTime" == name) gp_.autoOffTime(static_cast<AutoOffTime>(val));

            // Video only
            else if ("videoStreamBitRate" == name) gp_.videoStreamBitRate(static_cast<VideoStreamBitRate>(val)); 
            else if ("videoFrameRate" == name) gp_.videoFrameRate(static_cast<VideoFrameRate>(val));
            else if ("videoResolution" == name) gp_.videoResolution(static_cast<VideoResolution>(val));
            else if ("videoFrameRate" == name) gp_.videoFrameRate(static_cast<VideoFrameRate>(val));
            else if ("videoFOV" == name) gp_.videoFOV(static_cast<VideoFOV>(val));
            else if ("videoLowLight" == name) gp_.videoLowLight(val);
            else if ("videoLoopDuration" == name) gp_.videoLoopDuration(static_cast<VideoLoopDuration>(val));
            else if ("videoPhotoInterval" == name) gp_.videoPhotoInterval(static_cast<VideoPhotoInterval>(val));
            else if ("videoTagMoment" == name) gp_.videoTagMoment();

            // Multishot only
            else if ("multiBurstRate" == name) gp_.multiBurstRate(static_cast<MultiBurstRate>(val));
            else if ("multiTimeLapseInterval" == name) gp_.multiTimeLapseInterval(static_cast<MultiTimeLapseInterval>(val));
            else if ("multiNightLapseInterval" == name) gp_.multiNightLapseInterval(static_cast<MultiNightLapseInterval>(val));
            // Mode-specific
            else if ("whiteBalance" == name) gp_.whiteBalance(static_cast<WhiteBalance>(val));
            else if ("color" == name) gp_.color(static_cast<Color>(val));
            else if ("isoLimit" == name) gp_.isoLimit(static_cast<ISOLimit>(val));
            else if ("isoMin" == name) gp_.isoMin(static_cast<ISOMin>(val));
            else if ("sharpness" == name) gp_.sharpness(static_cast<Sharpness>(val));
            else if ("ev" == name) gp_.ev(static_cast<EV>(val));
            else if ("exposure" == name) gp_.exposure(static_cast<Exposure>(val));
            else if ("spotMeter" == name) gp_.spotMeter(static_cast<SpotMeter>(val));
            else if ("photoResolution" == name) gp_.photoResolution(static_cast<PhotoResolution>(val));
        }
    }


    // A convenience service for triggering the shutter, switching mode beforehand,
    // and receiving mode-specific outputs.
    bool GoProHeroNode::triggerShutterCB(gopro_hero_msgs::Shutter::Request& req,
                                         gopro_hero_msgs::Shutter::Response& rsp)
    {
        vector<vector<unsigned char> > images;
        gp_.setMode(req.multishot ? GoProHero::Mode::MULTISHOT : GoProHero::Mode::PHOTO);
        gp_.shutter(true);
        gp_.currentImages(images);
        
        // Convert image bytes to sensor_msgs/CompressedImage
        for (auto i : images)
        {
            sensor_msgs::CompressedImage rosImg;rosImg.format = "jpeg"; // TODO parameterize
            copy(i.begin(), i.end(), back_inserter(rosImg.data));
            rsp.images.push_back(rosImg);
        }
        
        return true;
    }


    // Function called in a new thread when streaming is started
    void GoProHeroNode::processStreamFrameCB(int width, int height, int numBytes, uint8_t* bytes)
    {
        ROS_INFO_STREAM("Received " << numBytes << " from stream.");
    }
    
}
