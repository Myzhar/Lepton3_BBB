#ifndef __VIDEO_ENCODER_H
#define __VIDEO_ENCODER_H

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <sstream>

/**
 * videoEncoder
 */
class videoEncoder
{
public:
    static videoEncoder* Create(uint32_t width, uint32_t height, std::string filename, std::string encName="Gst Stream" );
    static videoEncoder* Create(uint32_t width, uint32_t height, std::string multicastIface, std::string ipAddress, uint32_t port, uint32_t bitrate = 2000000, std::string encName="Gst Stream" );
    ~videoEncoder();

    bool Open();
    bool Close();

    bool PushFrame(uint8_t* buf ); // YUV format!!!

private:
    videoEncoder(uint32_t width, uint32_t height , std::string multicastIface, uint32_t bitrate, std::string encName );
    bool init();

    static void onNeed(_GstElement* pipeline, uint32_t size, void* user_data);
    static void onEnough(_GstElement* pipeline, void* user_data);

    bool buildLaunchStr();
    bool buildCapsStr();

    void msgOut(std::string message );

private:

    GstBus*     mBus;
    //GstCaps*    mBufferCaps;
    GstElement* mAppSrc;
    GstElement* mPipeline;
    GstBuffer*  mInputBuffer;
    bool         mNeedData;
    bool		 mOpened;

    std::string  mMulticastIface;
    std::string  mLaunchStr;
    std::string  mCapsStr;
    std::string  mOutputPath;
    std::string  mOutputIP;
    uint32_t     mOutputPort;

    uint32_t     mWidth;
    uint32_t     mHeight;

    uint32_t     mBitRate;

    std::string mEncName;
};



#endif

