#include "gopro_hero/gopro_hero_stream.hpp"

#include <boost/chrono.hpp>
#include <boost/exception/exception.hpp>

using namespace cv;
using namespace std;

/*! \file gopro_hero_stream.cpp
    \brief Streaming class for GoPro Hero camera
    \todo Remove Boost dependency
*/

namespace gopro_hero
{

    /// Primary constructor
    /// \param captureHost GoPro host address, ex. 10.5.5.9
    /// \param capturePort GoPro UDP comms port
    GoProHeroStream::GoProHeroStream(string captureHost,
                                     unsigned int capturePort) :
        captureHost_(captureHost),
        capturePort_(capturePort),
        errored_(false),
        pause_(true)
    {

    }

    /// Destructor
    ///
    GoProHeroStream::~GoProHeroStream()
    {
        keepAliveThread_.interrupt();
        videoCapture_->release();
        captureThread_.join();
    }


    /// Spawn the class' threads
    /// \return success indication of node spin up
    bool GoProHeroStream::start()
    {
        call_once(startOnceFlag_, [&](){
                pause_ = false;
                captureThread_ = boost::thread(&GoProHeroStream::captureThreadFunc, this);
                keepAliveThread_ = boost::thread(&GoProHeroStream::keepAliveThreadFunc, this, 2000);
            });
        
        return !errored_;
    }


    /// Forcibly stop the stream, reset all vars, start the stream
    /// \return success indication of restart
    bool GoProHeroStream::restart()
    {
        captureThread_.interrupt();
        keepAliveThread_.interrupt();
        errored_ = false;
        pause_ = false;
        captureThread_ = boost::thread(&GoProHeroStream::captureThreadFunc, this);
        keepAliveThread_ = boost::thread(&GoProHeroStream::keepAliveThreadFunc, this, 2000);
        return !errored_;
    }

    
    /// Place all running threads (except keepalive) in a holding state
    /// \param pause if true, pause; unpause otherwise
    void GoProHeroStream::pause(bool pause)
    {
        pause_ = pause;
    }


    /// Capture video frames while capture is not paused
    ///
    void GoProHeroStream::captureThreadFunc()
    {
        Mat frame;
        preCaptureCommands_();
        videoCapture_ = new VideoCapture("udp://" + captureHost_ + ":" + to_string(capturePort_));
        postCaptureCommands_();
        while (videoCapture_->isOpened())
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
            if (!pause_)
            {
                *videoCapture_ >> frame;
                captureCallbackFunc_(frame);
            }
        }
        errorCallbackFunc_("Video stream has been closed.");
        errored_ = true;
        return;
    }


    /// Error callback for boost async_send_to
    /// \param ec boost error code upon failed comms
    void GoProHeroStream::errorCB(const boost::system::error_code& ec)
    {
        string err = "Boost error code " + to_string(ec.value());
        errorCallbackFunc_(err);
    }


    /// Sends an essential "keep alive" message to maintain the GoPro's stream
    /// \param delayMS delay time between each sent packet in milliseconds
    void GoProHeroStream::keepAliveThreadFunc(unsigned int delayMS)
    {
        try
        {
            using namespace boost::asio;
            io_service ioService;
            ip::udp::resolver resolver(ioService);
            ip::udp::endpoint dest(ip::address::from_string(captureHost_), capturePort_);
            ip::udp::socket sock(ioService, ip::udp::v4());

            for (;;)
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(delayMS));
                sock.async_send_to(buffer("_GPHD_:0:0:2:0.000000\n", 22), dest,
                                   boost::bind(&GoProHeroStream::errorCB, this,
                                               boost::asio::placeholders::error));
            }
        }
        catch (boost::exception& e)
        {
            errorCallbackFunc_(boost::diagnostic_information(e));
        }
        errored_ = true;
    }

}
