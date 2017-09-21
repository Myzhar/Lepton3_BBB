#include "flir_tracker.h"
#include "Palettes.h"

#include <iostream>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

FlirTracker::FlirTracker(TrackMode trkMode, uint16_t minThresh, uint16_t maxThresh)
    : mTrkMode(trkMode)
    , mMinThresh(minThresh)
    , mMaxThresh(maxThresh)
{

}

FlirTracker::~FlirTracker()
{

}

FlirTracker::TrackRes FlirTracker::setNewFrame( cv::Mat frame16, uint16_t min, uint16_t max  )
{
    if( frame16.type() != CV_16U )
    {
        cerr << "Wrong image type. One channel 16bit image expected" << endl;
        return TRK_RES_ERROR;
    }

    if( frame16.channels() != 1 )
    {
        cerr << "Wrong image channel count. One channel 16bit image expected" << endl;
        return TRK_RES_ERROR;
    }

    mFrameMin = min;
    mFrameMax = max;

    mFrame16 = frame16;

    return doTrackStep();
}

FlirTracker::TrackRes FlirTracker::doTrackStep()
{
    cv::inRange( mFrame16, cv::Scalar(mMinThresh), cv::Scalar(mMaxThresh), mResMask );

    //cv::imshow( "Range mask", mResMask );

    mFrame16.copyTo(mRes16,mResMask); // Mask

    return TRK_RES_NONE;
}


cv::Mat FlirTracker::getResFrameRGB()
{
    /*if( !mResRGB.empty() )
        return cv::Mat();*/

    uint16_t max_val = static_cast<uint16_t>(pow(2,14)-1);
    cv::Mat temp8 = normalizeFrame( mFrame16, 0, max_val );

    cv::cvtColor( temp8, mResRGB, CV_GRAY2BGR );

    mResRGB.create(temp8.size(), CV_8UC3 );
    //cv::imshow( "Original_int", mResRGB );

    for( int r=0; r<mResRGB.rows; r++ )
    {
        for( int c=0; c<mResRGB.cols; c++ )
        {
            const uint8_t* lut = colormap_rainbow;

            int maskVal = mResMask.at<uint8_t>(r,c);
            if( maskVal!=0 )
            {
                uint16_t val = temp8.at<uint8_t>(r,c);
                uint8_t r_ = lut[val+2];
                uint8_t g_ = lut[val+1];
                uint8_t b_ = lut[val+0];
                int idx = 3*(r*160+c);
                mResRGB.data[idx+2] = r_;
                mResRGB.data[idx+1] = g_;
                mResRGB.data[idx+0] = b_;
            }
        }
        cout << endl;
    }



    return mResRGB;
}

cv::Mat FlirTracker::normalizeFrame( const cv::Mat& frame16, uint16_t min, uint16_t max )
{
    cv::Mat tmp16;
    frame16.copyTo( tmp16);

    // >>>>> Rescaling/Normalization to 8bit
    double diff = static_cast<double>(max - min); // Image range
    double scale = 255./diff; // Scale factor

    tmp16 -= min; // Bias
    tmp16 *= scale; // Rescale data

    cv::Mat frame8;
    tmp16.convertTo( frame8, CV_8UC1 );
    // <<<<< Rescaling/Normalization to 8bit

    return frame8;
}
