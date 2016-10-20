#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <typeinfo>
#include <sstream>
#include <iomanip>

#include <curl/curl.h>

#include "gopro_hero_commands.hpp"


namespace gopro_hero {

    class GoProHero {
    public:

        using Mode = PrimaryMode;

        GoProHero();
        ~GoProHero();

        void currentImages(std::vector<std::vector<unsigned char> >& images, long timeout = 10);
        void setMode(Mode m);


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

        
        void sendSetting(std::string s) { send(base_ + "setting/" + s); }
        void sendCommand(std::string s) { send(base_ + "command/" + s); }
        void sendMagicPacket(std::array<unsigned char, 6> mac);
        bool send(std::string s);
        
        bool curlGetBytes(const std::string url, std::vector<unsigned char>& image, long timeout = 10);
        static size_t curlWriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
        bool curlGetText(const std::string url, std::string& text, long timeout = 10);
        bool curlRequestUrl(const std::string url, std::string& readBuffer, long timeout = 10);
        
        std::string zeroPaddedIntString(std::string num, int pad);
        

        const std::string base_ = GoProHeroCommands::commandBase();
        Mode mode_;
        bool saveOnDevice_;
    };
}

#endif
