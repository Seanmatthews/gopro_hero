#include "gopro_hero/gopro_hero_stream.hpp"

#include <boost/chrono.hpp>
#include <boost/exception/exception.hpp>

using namespace cv;
using namespace std;


namespace rowboat1
{
    
    GoProHeroStream::GoProHeroStream(string captureHost,
                                     unsigned int capturePort) :
        captureHost_(captureHost),
        capturePort_(capturePort),
        errored_(false),
        pause_(true)
    {

    }

    
    GoProHeroStream::~GoProHeroStream()
    {
        keepAliveThread_.interrupt();
        videoCapture_->release();
        captureThread_.join();
    }

    
    // Spawn the class' threads
    bool GoProHeroStream::start()
    {
        pause_ = false;
        boost::call_once([&](){
                captureThread_ = boost::thread(&GoProHeroStream::captureThreadFunc, this);
                keepAliveThread_ = boost::thread(&GoProHeroStream::keepAliveThreadFunc, this, 2000);
            }, startOnceFlag_);
        
        return errored_;
    }

    
    // Place all running threads in a holding state,
    // apart from the keep alive thread
    void GoProHeroStream::pause()
    {
        pause_ = true;
    }


    // Capture video frames while capture is not paused
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


    // Error callback for boost async_send_to
    void GoProHeroStream::errorCB(const boost::system::error_code& ec)
    {
        string err = "Boost error code " + to_string(ec.value());
        errorCallbackFunc_(err);
    }


    // Sends an essential "keep alive" message to maintain the GoPro's stream
    void GoProHeroStream::keepAliveThreadFunc(unsigned int delayMS)
    {
        try
        {
            boost::asio::io_service ioService;
            boost::asio::ip::udp::resolver resolver(ioService);
            boost::asio::ip::udp::endpoint dest(boost::asio::ip::address::from_string(captureHost_), capturePort_);
            boost::asio::ip::udp::socket sock(ioService, boost::asio::ip::udp::v4());

            for (;;)
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(delayMS));
                sock.async_send_to(boost::asio::buffer("_GPHD_:0:0:2:0.000000\n", 22), dest,
                                   boost::bind(&GoProHeroStream::errorCB, this, boost::asio::placeholders::error));
            }
        }
        catch (boost::exception& e)
        {
            errorCallbackFunc_(boost::diagnostic_information(e));
        }
        errored_ = true;
    }

}
