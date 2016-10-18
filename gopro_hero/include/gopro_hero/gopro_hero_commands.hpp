#ifndef GOPRO_HERO_COMMANDS_H_
#define GOPRO_HERO_COMMANDS_H_

#include <string>
#include <map>
#include <typeinfo>

namespace rowboat1 {

    enum class WhiteBalance
    {
        AUTO = 0,
        K3000 = 1,
        K4000 = 5,
        K4800 = 6,
        K5500 = 2,
        K6000 = 7,
        K6500 = 3,
        NATIVE = 4
    };

    enum class Orientation
    {
        GYRO = 0,
        UP = 1,
        DOWN = 2
    };

    enum class Color
    {
        GOPRO = 0,
        FLAT = 1
    };

    enum class ISOLimit
    {
        I6400 = 0,
        I1600 = 1,
        I400 = 2,
        I3200 = 3,
        I800 = 4,
        I200 = 7,
        I100 = 8
    };

    enum class Sharpness
    {
        HIGH = 0,
        MED = 1,
        LOW = 2
    };

    enum class EV 
    {
        PLUS2 = 0,
        PLUS1_5 = 1,
        PLUS1 = 2,
        PLUS0_5 = 3,
        ZERO = 4,
        NEG0_5 = 5,
        NEG1 = 6,
        NEG1_5 = 7,
        NEG2 = 8
    };

    enum class Control
    {
        ON = 0,
        OFF = 1
    };

    enum class ISOMode
    {
        MAX = 0,
        LOCK = 1
    };

    enum class ISOMin
    {
        I800 = 0,
        I400 = 1,
        I200 = 2,
        I100 = 3
    };

    enum class Exposure
    {
        FPS24_24 = 3,    FPS24_48 = 6,    FPS24_96 = 11,
        FPS30_30 = 5,    FPS30_60 = 8,    FPS30_120 = 13,
        FPS48_48 = 6,    FPS48_96 = 11,   FPS48_192 = 16,
        FPS60_60 = 8,    FPS60_120 = 13,  FPS60_240 = 18,
        FPS90_90 = 10,   FPS90_180 = 15,  FPS90_360 = 20,
        FPS120_120 = 13, FPS120_240 = 18, FPS120_480 = 22,
        FPS240_120 = 18, FPS240_240 = 22, FPS240_480 = 23
    };

    enum class DefaultBootMode
    {
        VIDEO = 0,
        PHOTO = 1,
        MULTISHOT = 2
    };

    enum class PrimaryMode
    {
        VIDEO = 0,
        PHOTO = 1,
        MULTISHOT = 2
    };

    enum class VideoSecondaryMode
    {
        VIDEO = 0,
        TIMELAPSE = 1,
        PHOTO = 2,
        LOOPING = 3,
    };

    enum class PhotoSecondaryMode
    {
        SINGLE = 0,
        CONTINUOUS = 1,
        NIGHT = 2
    };

    enum class MultishotSecondaryMode
    {
        BURST = 0,
        TIMELAPSE = 1,
        NIGHTLAPSE = 2
    };

    enum class MultiBurstRate
    {
        B3_1 = 0,
        B5_1 = 1,
        B10_1 = 2,
        B10_2 = 3,
        B10_3 = 4,
        B30_1 = 5,
        B30_2 = 6,
        B30_3 = 7,
        B30_6 = 8
    };
    
    enum class VideoResolution
    {
        K4 = 1,
        K4_SUPERVIEW = 2,
        K27 = 4,
        K27_SUPERVIEW = 5,
        K27_43 = 6,
        P1440 = 7,
        P1080_SUPERVIEW = 8,
        P1080 = 9,
        P960 = 10,
        P720_SUPERVIEW = 11,
        P720 = 12,
        WVGA = 13
    };

    enum class PhotoResolution
    {
        WIDE_12MP = 0,
        WIDE_7MP = 1,
        STANDARD_7MP = 2,
        WIDE_5MP = 3
    };

    enum class VideoFrameRate
    {
        FPS240 = 0,
        FPS120 = 1,
        FPS100 = 2,
        FPS60 = 3,
        FPS50 = 6,
        FPS48 = 7,
        FPS30 = 8,
        FPS25 = 9,
        FPS24 = 10,
        FPS15 = 11,
        FPS12_5 = 12
    };

    enum class VideoFOV
    {
        WIDE = 0,
        MEDIUM = 1,
        NARROW = 2
    };

    enum class VideoLoopDuration
    {
        MAX = 0,
        MIN5 = 1,
        MIN20 = 2,
        MIN60 = 3,
        MIN120 = 4
    };

    enum class VideoPhotoInterval
    {
        INT5 = 1,
        INT10 = 2,
        INT30 = 3,
        INT60 = 4
    };

    enum class SpotMeter
    {
        SPOTOFF = 0,
        SPOTON = 1
    };

    enum class LEDBlink
    {
        OFF = 0,
        BLINK2 = 1,
        BLINK4 = 2
    };

    enum class BeepVolume
    {
        FULL = 0,
        HALF = 1,
        OFF = 2
    };

    enum class LCDBrightness
    {
        HIGH = 0,
        MEDIUM = 1,
        LOW = 2
    };

    enum class LCDSleepTimeout
    {
        NEVER = 0,
        M1 = 1,
        M2 = 2,
        M3 = 3,
        M5 = 4
    };

    enum class AutoOffTime
    {
        NEVER = 0,
        M1 = 1,
        M2 = 2,
        M3 = 3,
        M5 = 4
    };

    enum class VideoStreamBitRate
    {
        K250 = 250000,
        K400 = 400000,
        K600 = 600000,
        K700 = 700000,
        K800 = 800000,
        M1 = 1000000,
        M1_2 = 1200000,
        M1_6 = 1600000,
        M2 = 2000000,
        M2_4 = 2400000
    };

    enum class VideoStreamWindowSize
    {
        DEFAULT = 0,
        W240 = 1,
        W240_34 = 2,
        W240_12 = 3,
        W480 = 4,
        W480_34 = 5,
        W480_12 = 6
    };

    enum class MultiTimeLapseInterval
    {
        S0_5 = 0,
        S1 = 1,
        S2 = 2,
        S5 = 5,
        S10 = 10,
        S30 = 30,
        S60 = 60
    };

    enum class MultiNightLapseInterval
    {
        CONTINUOUS = 0,
        S4 = 4,
        S5 = 5,
        S10 = 10,
        S15 = 15,
        S20 = 20,
        S30 = 30,
        M1 = 60,
        M2 = 120,
        M5 = 300,
        M30 = 1800,
        M60 = 3600
    };
    

    class GoProHeroCommands {
    public:
        
        //
        template<typename T>
        static constexpr auto to_type(T x) -> typename std::underlying_type<T>::type {
            return static_cast<typename std::underlying_type<T>::type>(x);
        }

        // 
        template<typename T>
        static std::string to_string(T x) {
            return std::to_string(GoProHeroCommands::to_type(x));
        }

        // 
        static const std::string commandBase() { return "http://10.5.5.9/gp/gpControl/"; }

        static const std::map<std::string, std::string> videoModeVals() {
            return {
                {typeid(WhiteBalance).name(), "11/"},
                {typeid(Color).name(), "12/"},
                {typeid(ISOLimit).name(), "13/"},
                {typeid(Sharpness).name(), "14/"},
                {typeid(EV).name(), "15/"}
            };
        }
   
        static const std::map<std::string, std::string> photoModeVals() {
            return {};
        }
        
        static const std::map<std::string, std::string> multiModeVals() {
            return {};
        }
    };

}

#endif
