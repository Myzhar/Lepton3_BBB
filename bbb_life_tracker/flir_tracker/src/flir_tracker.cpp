#include "flir_tracker.h"
#include "Palettes.h"

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define RAW2CELSIUS 0.009

FlirTracker::FlirTracker(TrackMode trkMode, uint16_t minThresh, uint16_t maxThresh, 
	uint8_t rowMin/*=40*/, uint8_t rowMax/*=100*/)
    : mTrkMode(trkMode)
    , mMinThresh(minThresh)
    , mMaxThresh(maxThresh)
{
    mPaletteIdx = 6;

    createColorMaps();
    mRowMin = rowMin;
    mRowMax = rowMax;
    
    mTargHist = cv::Mat( 160, 1, CV_64F );
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

    //frame16.copyTo(mFrame16);
    mFrame16 = frame16;

    return doTrackStep();
}

void FlirTracker::setNewThresh(uint16_t minThresh, uint16_t maxThresh)
{
    mMinThresh = minThresh;
    mMaxThresh = maxThresh;
    
    cout << "New MIN thresh: " << (int)mMinThresh << " (" << static_cast<double>(mMinThresh)*RAW2CELSIUS <<
            "°C) - New MAX thresh: " << mMaxThresh << " (" << static_cast<double>(mMaxThresh)*RAW2CELSIUS 
            << "°C)\r" << endl;
}

FlirTracker::TrackRes FlirTracker::doTrackStep()
{
    cv::Rect roi;
    roi.x = 0;
    roi.width = 160;
    roi.y = mRowMin;
    roi.height = mRowMax-mRowMin;
    
    cv::Mat frm16Cropped = mFrame16(roi);        
    cv::inRange( mFrame16, mMinThresh, mMaxThresh, mResMask );
    
    double maxHist = static_cast<double>(mRowMax-mRowMin);
    double meanX = 0.0;
    
    double max = 0.0;
    double sum = 0.0;    
    
    // >>>>> Target Histogram
    for( int c=0; c<160; c++ )
    {
        double colValue=0.0;       
        
        for( int r=mRowMin; r<mRowMax; r++ )
        {            
            uint8_t maskVal = mResMask.at<uint8_t>(r,c);
            if( maskVal!=0 )
            {
                colValue+=1.0;
            }
        }
        
        colValue /= maxHist;
        if(colValue>max)
            max=colValue;
        mTargHist.at<double>(c) = colValue;        
        sum += colValue;
    }
    // <<<<< Target Histogram
    
    // >>>>> Histogram normalization and target position
    int count = 0;
    mTargHist /= sum;
    
    for( int c=0; c<160; c++ )
    {
        double colValue = mTargHist.at<double>(c);
        meanX += (c+1)*colValue;        
    }
    
    mTargetPos.x = static_cast<int>(meanX);
    mTargetPos.y = 60;
    // >>>>> Histogram normalization and target position    
    
    // Histogram de-normalization for rendering
    mTargHist *= sum;//cout << mResMask.cols << "x" << mResMask.rows << "\r\n";
    
    if(sum>0)
        return TRK_RES_FOUND;
    else
        return TRK_RES_NONE;
}

void FlirTracker::nextPalette()
{ 
    mPaletteIdx = (mPaletteIdx+1)%PALETTES_COUNT;

    cout << "New palette: " << (int)mPaletteIdx << "\r" << endl;
}

cv::Mat FlirTracker::getResFrameRGB()
{
    /*if( !mResRGB.empty() )
        return cv::Mat();*/

    cv::Mat temp8 = normalizeFrame( mFrame16, mFrameMin, mFrameMax );

    cv::cvtColor( temp8, mResRGB, CV_GRAY2BGR );

    //mResRGB.create(temp8.size(), CV_8UC3 );
    //cv::imshow( "Original_int", mResRGB );
    
    const uint8_t* lut = palettes[mPaletteIdx];

    //for( int r=0; r<mResRGB.rows; r++ )
    for( int r=mRowMin; r<mRowMax; r++ )
    {
        for( int c=0; c<mResRGB.cols; c++ )
        {
            uint8_t maskVal = mResMask.at<uint8_t>(r,c);
            if( maskVal!=0 )
            {
                uint16_t val = temp8.at<uint8_t>(r,c);
                
                uint8_t r_ = lut[val*3+0];
                uint8_t g_ = lut[val*3+1];
                uint8_t b_ = lut[val*3+2];
                int idx = 3*(r*160+c);
                mResRGB.data[idx+0] = r_;
                mResRGB.data[idx+1] = g_;
                mResRGB.data[idx+2] = b_;
            }
        }
    }//*/
    
    cv::line( mResRGB, cv::Point(0,mRowMin), cv::Point(159,mRowMin), cv::Scalar(255,255,255), 1  );
    cv::line( mResRGB, cv::Point(0,mRowMax), cv::Point(159,mRowMax), cv::Scalar(255,255,255), 1  );
    
    int w = 2*(mRowMax-mRowMin)/3;
       
    
    string txt;
    if(mTrkMode == TRK_FOLLOW)
    { 
        txt="TARGET FOLLOW";
        
        cv::line( mResRGB, cv::Point((160-w)/2,mRowMin), cv::Point((160-w)/2,mRowMax), cv::Scalar(255,255,255), 1  );
        cv::line( mResRGB, cv::Point((160+w)/2,mRowMin), cv::Point((160+w)/2,mRowMax), cv::Scalar(255,255,255), 1  );
    }
    else
    {
        txt="OBSTACLE AVOIDANCE";
        
        cv::line( mResRGB, cv::Point(w,mRowMin), cv::Point(w,mRowMax), cv::Scalar(255,255,255), 1  );
        cv::line( mResRGB, cv::Point(160-w,mRowMin), cv::Point(160-w,mRowMax), cv::Scalar(255,255,255), 1  );
    }
    
    cv::putText( mResRGB, txt, cv::Point( 2,118 ), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255,255,255), 1 );
    
    // Target
    cv::circle( mResRGB, mTargetPos, 2, cv::Scalar(255,255,255), -1 );
    
    // >>>>> Target Histogram
    int histY = 107;
    int histH = (histY-mRowMax)-2;
    for( int c=0; c<160; c++ )
    {
        double colValue = mTargHist.at<double>(c);        
        int h = colValue * histH;
        
        double dist = fabs(c-mTargetPos.x);
        double norm = dist>40?1.0:dist/40.0;
        
        cv::Scalar color = cv::Scalar(255,255.0*norm,100);
       
        cv::line( mResRGB, cv::Point(c,histY), cv::Point(c,histY-h), color, 1  );
    }
    // <<<<< Target Histogram */

    //cv::cvtColor( mResMask, mResRGB, CV_GRAY2BGR );
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
