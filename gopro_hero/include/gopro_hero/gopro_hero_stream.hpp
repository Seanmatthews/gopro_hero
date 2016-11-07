#ifndef GOPROHEROSTREAM_HPP_
#define GOPROHEROSTREAM_HPP_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"

#include <mutex>
#include <thread>
#include <string>


namespace gopro_hero
{
    
class GoProHeroStream
{
public:
    GoProHeroStream(std::string captureHost,
                    unsigned int capturePort);
    ~GoProHeroStream();

    bool start();
    bool restart();
    void pause(bool pause);
    
    void registerCaptureCallback(std::function<void(cv::Mat& img)> func)
        { captureCallbackFunc_ = func; }

    void registerErrorCallback(std::function<void(const std::string err)> func)
        { errorCallbackFunc_ = func; }

    void setPreCaptureCommands(std::function<void()> func)
        { preCaptureCommands_ = func; }

    void setPostCaptureCommands(std::function<void()> func)
        { postCaptureCommands_ = func; }

private:
    void captureThreadFunc();
    void errorCB(const boost::system::error_code& sc);
    void keepAliveThreadFunc(unsigned int delayMS);

    std::function<void(cv::Mat& img)> captureCallbackFunc_;
    std::function<void(const std::string err)> errorCallbackFunc_;
    std::function<void()> preCaptureCommands_, postCaptureCommands_;
    std::string captureHost_;
    unsigned int capturePort_;
    cv::VideoCapture* videoCapture_;
    boost::thread captureThread_, keepAliveThread_;
    bool errored_, pause_;
    std::once_flag startOnceFlag_;
};
}

#endif 
