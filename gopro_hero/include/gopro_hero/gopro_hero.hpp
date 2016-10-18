#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <map>
#include <typeinfo>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <curl/curl.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>

extern "C"
{
#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
}

#include "gopro_hero_commands.hpp"


namespace rowboat1 {

    class GoProHero {
    public:

        using Mode = PrimaryMode;

        GoProHero() :
            isStreaming_(false),
            saveOnDevice_(true) {
            mode_ = Mode::PHOTO;
            curl_global_init(CURL_GLOBAL_ALL);
        }
        
        ~GoProHero() {
            curl_global_cleanup();
        }

        // Sets an external frame processor 
//        void setStreamFrameCallback( void(*func)(int,int,int,uint8_t*) ) { processFrameFunc_ = func; }
        
        bool isStreaming() {
            return isStreaming_;
        }
        
        std::string zeroPaddedIntString(std::string num, int pad) {
            std::ostringstream ss;
            ss << std::setw(pad) << std::setfill('0') << num;
            return ss.str();
        }



        // Lazy get -- move boilerplate to util function
        void currentImages(std::vector<std::vector<unsigned char> >& images, long timeout = 10) {
            Json::Value root;
            Json::Reader reader;
            std::string mediaList;

            if (!curlGetText("http://10.5.5.9/gp/gpMediaList", mediaList, 2)) return;
            std::cout << mediaList << std::endl;

            if (!mediaList.empty() && reader.parse(mediaList, root))
            {
                const Json::Value media = root["media"][0]["fs"];
                const Json::Value lastVal = media[media.size() - 1];

                // TODO Check that it's a JPG
                
                int startNum = std::stoi(lastVal["b"].asString());
                int endNum = std::stoi(lastVal["l"].asString());
                for (int i=startNum; i<=endNum; ++i)
                {
                    std::string path = "http://10.5.5.9/videos/DCIM/100GOPRO/G" +
                        zeroPaddedIntString(lastVal["g"].asString(), 3) +
                        zeroPaddedIntString(std::to_string(i), 4) + ".JPG";
                    
                    std::vector<unsigned char> image;
                    curlGetBytes(path, image, timeout);
                    images.push_back(image);
                }
                                
            }
        }

        // Is this correct? Setting mode == turning mode on?
        void setMode(Mode m) {
            mode_ = m;
            switch (m) {
            case Mode::VIDEO:
            {
                sendCommand("mode?p=0"); // set as mode
                sendSetting("10/1"); // turn on
                break;
            }
            case Mode::PHOTO:
            {
                sendCommand("mode?p=1");
                sendSetting("21/1");
                break;
            }
            case Mode::MULTISHOT:
            {
                sendCommand("mode?p=2");
                sendSetting("34/1");
                break;
            }
            default: break;
            }
        }

        // Global functions
        void shutter(bool on) { sendCommand("shutter?p=" + std::to_string((on ? 1 : 0))); }
        void orientation(Orientation o) { sendSetting("52/" + GoProHeroCommands::to_string(o)); }
        void ledBlink(LEDBlink b) { sendSetting("55/" + GoProHeroCommands::to_string(b)); }
        void beepVolume(BeepVolume b) { sendSetting("56/" + GoProHeroCommands::to_string(b)); }
        void lcdDisplay(bool on) { sendSetting("72/" + std::to_string(on ? 1 : 0)); }
        void onScreenDisplay(bool on) { sendSetting("58/" + std::to_string(on ? 1 : 0)); }
        void lcdBrightness(LCDBrightness b) { sendSetting("49/" + GoProHeroCommands::to_string(b)); }
        void lcdLock(bool on) { sendSetting("50/" + std::to_string(on ? 1 : 0)); }
        void lcdSleepTimeout(LCDSleepTimeout t) { sendSetting("51/" + GoProHeroCommands::to_string(t)); }
        void autoOffTime(AutoOffTime a) { sendSetting("59/" + GoProHeroCommands::to_string(a)); }
        void defaultBootMode(DefaultBootMode d) { sendSetting("53/" + GoProHeroCommands::to_string(d)); }
        void saveMediaOnDevice(bool yes) { saveOnDevice_ = yes; }
        void deleteLastTaken() { sendCommand("storage/delete/last"); }
        void deleteAllMedia() { sendCommand("storage/delete/all"); }
        void locate(bool on) { sendCommand("system/locate?p=" + std::to_string(on ? 1 : 0)); }
        void power(bool on, std::array<unsigned char, 6> mac = {}) {
            if (on) sendMagicPacket(mac);
            else sendCommand("system/sleep");
        }
        
        // Single mode functions
        void videoStreamStart() { send(base_ + "execute/?p1=gpStream&a1=proto_v2&c1=restart"); }
        void videoStreamBitRate(VideoStreamBitRate s) { sendSetting("62/" + GoProHeroCommands::to_string(s)); }
        void videoStreamWindowSize(VideoStreamWindowSize s) { sendSetting("64/" + GoProHeroCommands::to_string(s)); }
        void videoResolution(VideoResolution v) { sendSetting("2/" + GoProHeroCommands::to_string(v)); }
        void videoFrameRate(VideoFrameRate f) { sendSetting("3/" + GoProHeroCommands::to_string(f)); }
        void videoFOV(VideoFOV f) { sendSetting("4/" + GoProHeroCommands::to_string(f)); }
        void videoLowLight(bool on) { sendSetting("8/" + std::to_string(on ? 1 : 0)); }
        void videoLoopDuration(VideoLoopDuration v) { sendSetting("6/" + GoProHeroCommands::to_string(v)); }
        void videoPhotoInterval(VideoPhotoInterval v) { sendSetting("7/" + GoProHeroCommands::to_string(v)); }
        void videoTagMoment() { sendCommand("storage/tag_moment"); }
        void multiBurstRate(MultiBurstRate m) { sendSetting("29/" + GoProHeroCommands::to_string(m)); }
        void multiTimeLapseInterval(MultiTimeLapseInterval m) { sendSetting("31/" + GoProHeroCommands::to_string(m)); }
        void multiNightLapseInterval(MultiNightLapseInterval m) { sendSetting("32/" + GoProHeroCommands::to_string(m)); }
        
        // Mode-specific settings -- depend on current mode
        void whiteBalance(WhiteBalance w) { sendModalSetting(w); }
        void color(Color c) { sendModalSetting(c); }
        void isoLimit(ISOLimit i) { sendModalSetting(i); }
        void isoMin(ISOMin i) { sendModalSetting(i); }
        void sharpness(Sharpness s) { sendModalSetting(s); }
        void ev(EV e) { sendModalSetting(e); }
        void exposure(Exposure e) { sendModalSetting(e); }
        void spotMeter(SpotMeter s) { sendModalSetting(s); }
        void photoResolution(PhotoResolution p) { sendModalSetting(p); }
        
            
    private:
        template<typename T>
        void sendModalSetting(T s) {
            switch (mode_) {
            case Mode::VIDEO:
            {
                auto it = GoProHeroCommands::videoModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::videoModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::PHOTO:
            {
                auto it = GoProHeroCommands::photoModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::photoModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::MULTISHOT:
            {
                auto it = GoProHeroCommands::multiModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::multiModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            default:
                break;
            }
        }

        void sendMagicPacket(std::array<unsigned char, 6> mac) {
            using namespace boost::asio;

            std::array<unsigned char, 102> buf;

            for (int i=0; i<6; ++i) buf[i] = 0xFF; // 6 bytes
            for (int i=1; i<17; ++i) memcpy(&buf[i*6], &mac, 6 * sizeof(unsigned char)); // 96 bytes

            // send as UDP packet
            io_service ioService;
            ip::udp::socket socket(ioService);
            ip::udp::endpoint remoteEndpoint;
//            boost::system::error::error_code err;
            
            socket.open(ip::udp::v4());
            remoteEndpoint = ip::udp::endpoint(ip::address::from_string("10.5.5.9"), 9);
            socket.send_to(buffer(buf), remoteEndpoint); //, 0, err);
            socket.close();
        }
        
        void sendSetting(std::string s) { send(base_ + "setting/" + s); }
        void sendCommand(std::string s) { send(base_ + "command/" + s); }

        // TODO accept and parse output for success/failure--
        // maybe have this func return actual string, then sendCommand/sendSetting
        // can parse based on their respectie expected outputs.
        // TODO catch exceptions
        bool send(std::string s) {
            std::string empty;
            curlGetText(s, empty, 2);
            return true;
        }


        bool curlGetBytes(const std::string url, std::vector<unsigned char>& image, long timeout = 10) {
            std::string s;
            if (curlRequestUrl(url, s, timeout))
            {
                std::copy(s.begin(), s.end(), std::back_inserter(image));
                return true;
            }
            return false;
        }

        
        static size_t curlWriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
            ((std::string*)userp)->append((char*)contents, size * nmemb);
            return size * nmemb;
        }

        
        bool curlGetText(const std::string url, std::string& text, long timeout = 10) {
            return curlRequestUrl(url, text);
        }
        
        bool curlRequestUrl(const std::string url, std::string& readBuffer, long timeout = 10) {
            CURL* curl = curl_easy_init();
            CURLcode res(CURLE_FAILED_INIT);

            if(curl) {
                curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &GoProHero::curlWriteCallback);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
                curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
                res = curl_easy_perform(curl);
                curl_easy_cleanup(curl);

                std::cout << readBuffer.size() << std::endl;
            }
            return true; //CURLE_OK == res;
        }


    public:
        
        // http://hasanaga.info/tag/ffmpeg-libavcodec-av_read_frame-example/
        // http://stackoverflow.com/questions/10715170/receiving-rtsp-stream-using-ffmpeg-library
        static void streamThreadFunc( void (*processFrameFunc)(int,int,int,uint8_t*) ) {
            AVCodecContext *pCodecCtx;
            AVFormatContext *pFormatCtx = NULL;
            AVCodec *pCodec;
            AVFrame *pFrame, *pFrameRGB;
            AVPixelFormat pFormat = AV_PIX_FMT_BGR24;
            AVPacket packet;
            int videoStream = -1;
            uint8_t *buffer;
            int numBytes;
            int res;
            int frameFinished;
            std::string src = "udp://:8554";

            try
            {
                boost::this_thread::disable_interruption di1;
                
                av_register_all();
                avdevice_register_all();
                avcodec_register_all();
                avformat_network_init();
                
                if (avformat_open_input(&pFormatCtx, src.c_str(), NULL, NULL) != 0) return;
                if (avformat_find_stream_info(pFormatCtx, NULL) < 0) return;
                
                av_dump_format(pFormatCtx, 0, src.c_str(), 0);
                for (int i=0; i<pFormatCtx->nb_streams; ++i)
                {
                    if (pFormatCtx->streams[i]->codec->coder_type == AVMEDIA_TYPE_VIDEO)
                    {
                        videoStream = i;
                        break;
                    }
                }
                
                if (videoStream == -1) return;
                
                pCodecCtx = pFormatCtx->streams[videoStream]->codec;
                pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
                
                if (pCodec == NULL) return;
                if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) return;
                
                pFrame = avcodec_alloc_frame();
                pFrameRGB = avcodec_alloc_frame();
                
                numBytes = avpicture_get_size(pFormat, pCodecCtx->width, pCodecCtx->height);
                buffer = (uint8_t*)av_malloc(numBytes*sizeof(uint8_t));
                avpicture_fill((AVPicture*)pFrameRGB, buffer, pFormat, pCodecCtx->width, pCodecCtx->height);
                
                
                while(res = av_read_frame(pFormatCtx,&packet)>=0)
                {
                    boost::this_thread::restore_interruption ri(di1);
                    boost::this_thread::interruption_point();
                    {
                        boost::this_thread::disable_interruption di2;
                    
                        if(packet.stream_index == videoStream)
                        {
                            avcodec_decode_video2(pCodecCtx,pFrame,&frameFinished,&packet);
                            
                            if(frameFinished)
                            {
                                struct SwsContext * img_convert_ctx;
                                img_convert_ctx = sws_getCachedContext(NULL,pCodecCtx->width, pCodecCtx->height,
                                                                       pCodecCtx->pix_fmt, pCodecCtx->width,
                                                                       pCodecCtx->height, AV_PIX_FMT_BGR24,
                                                                       SWS_BICUBIC, NULL, NULL,NULL);
                                sws_scale(img_convert_ctx, ((AVPicture*)pFrame)->data,
                                          ((AVPicture*)pFrame)->linesize, 0, pCodecCtx->height,
                                          ((AVPicture *)pFrameRGB)->data, ((AVPicture *)pFrameRGB)->linesize);
                                
                                // Callback function set by parent
                                processFrameFunc(pFrame->height, pFrame->width, numBytes, pFrameRGB->data[0]);
                                
                                av_free_packet(&packet);
                                sws_freeContext(img_convert_ctx);
                            }
                        }
                    }
                }
            }
            catch (boost::thread_interrupted const&)
            {
                av_free_packet(&packet);
                avcodec_close(pCodecCtx);
                av_free(pFrame);
                av_free(pFrameRGB);
                avformat_close_input(&pFormatCtx);
            }
        }
        



    private:        
        const std::string base_ = GoProHeroCommands::commandBase();
        Mode mode_;
        bool isStreaming_;
        bool saveOnDevice_;
//        void (*processFrameFunc_)(int,int,int,uint8_t*);
    };
}

#endif
