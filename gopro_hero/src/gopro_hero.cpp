#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>

#include <iostream>

#include "gopro_hero/gopro_hero.hpp"

using namespace std;

/*! \file gopro_hero.cpp
    \brief Controls all camera functionality except for streaming
*/

namespace gopro_hero
{

    /// Constructor
    /// \note enable curl
    GoProHero::GoProHero() :
        saveOnDevice_(true), 
        mode_(Mode::PHOTO)
    {
        curl_global_init(CURL_GLOBAL_ALL);
    }


    /// Destructor
    /// \note clean up curl
    GoProHero::~GoProHero()
    {
        curl_global_cleanup();
    }


    /// Retrieves a list of images from the camera
    /// \param images the list of images, each as a vector of bytes
    /// \param timeout the amount of time to wait for the images
    /// \todo verify that images are jpegs
    void GoProHero::currentImages(vector<vector<unsigned char> >& images, long timeout) {
        Json::Value root;
        Json::Reader reader;
        std::string mediaList;
        
        if (!curlGetText("http://10.5.5.9/gp/gpMediaList", mediaList, 2)) return;
        cout << mediaList << endl;

        if (!mediaList.empty() && reader.parse(mediaList, root))
        {
            const Json::Value media = root["media"][0]["fs"];
            const Json::Value lastVal = media[media.size() - 1];
            
            // TODO Check that it's a JPG
            
            int startNum = stoi(lastVal["b"].asString());
            int endNum = stoi(lastVal["l"].asString());
            for (int i=startNum; i<=endNum; ++i)
            {
                string path = "http://10.5.5.9/videos/DCIM/100GOPRO/G" +
                    zeroPaddedIntString(lastVal["g"].asString(), 3) +
                    zeroPaddedIntString(to_string(i), 4) + ".JPG";
                    
                vector<unsigned char> image;
                curlGetBytes(path, image, timeout);
                images.push_back(image);
            }
        }
    }


    /// Set the camera's primary mode (video, photo, multishot)
    /// \param m PrimaryMode enum class
    void GoProHero::setMode(Mode m)
    {
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


    /// Sends a wake-on-lan packet to the camera
    /// \param mac mac address of YOUR network adapter
    /// \link https://en.wikipedia.org/wiki/Wake-on-LAN
    void GoProHero::sendMagicPacket(array<unsigned char, 6> mac)
    {
        using namespace boost::asio;
        
        array<unsigned char, 102> buf;
        for (int i=0; i<6; ++i) buf[i] = 0xFF; // 6 bytes
        for (int i=1; i<17; ++i) memcpy(&buf[i*6], &mac, 6 * sizeof(unsigned char)); // 96 bytes
        
        // send as UDP packet
        io_service ioService;
        ip::udp::socket socket(ioService);
        ip::udp::endpoint remoteEndpoint;
            
        socket.open(ip::udp::v4());
        remoteEndpoint = ip::udp::endpoint(ip::address::from_string("10.5.5.9"), 9);
        socket.send_to(buffer(buf), remoteEndpoint); //, 0, err);
        socket.close();
    }

    /// Sends a string of bytes to the camera
    /// \param a string that holds an array of bytes
    /// \return success condition
    /// \todo Accept and parse output for success/failure--
    /// \todo Catch exceptions
    bool GoProHero::send(string s)
    {
        string empty;
        curlGetText(s, empty, 2);
        return true;
    }


    /// Get a list of images from the camera
    /// \param url the camera's control/setting url
    /// \param image a vector of images (byte arrays)
    /// \param timeout attempt the read for this many ms
    /// \return success condition
    bool GoProHero::curlGetBytes(const string url, vector<unsigned char>& image, long timeout)
    {
        string s;
        if (curlRequestUrl(url, s, timeout))
        {
            copy(s.begin(), s.end(), back_inserter(image));
            return true;
        }
        return false;
    }


    /// A curl-specific callback for reading packets
    /// \return the size of the packet read
    size_t GoProHero::curlWriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
        ((string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }


    /// Retrieve text from a requested url
    /// \param url the command/setting url
    /// \param text the retrieved text
    /// \param timeout the amount of time to wait for request response
    /// \return success condition
    bool GoProHero::curlGetText(const string url, string& text, long timeout)
    {
        return curlRequestUrl(url, text, timeout);
    }


    /// Request a url and fill a buffer with the response
    /// \param url the command/setting url
    /// \param readBuffer the buffer into which to put the response
    /// \param timeout the amount of time to wait for response
    /// \return success condition
    bool GoProHero::curlRequestUrl(const string url, string& readBuffer, long timeout)
    {
        CURL* curl = curl_easy_init();
        CURLcode res(CURLE_FAILED_INIT);
        
        if (curl)
        {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &GoProHero::curlWriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
            curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
            res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);
        }
        return true; //CURLE_OK == res;
    }


    /// Pad a string with zeroes at the beginning
    /// \param num the string to pad
    /// \param pad the number of zeroes
    /// \return the new padded string
    string GoProHero::zeroPaddedIntString(string num, int pad)
    {
        ostringstream ss;
        ss << setw(pad) << setfill('0') << num;
        return ss.str();
    }    
    
}
