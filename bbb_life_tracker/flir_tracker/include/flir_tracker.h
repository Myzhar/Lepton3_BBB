#ifndef FLIR_TRACKER_H
#define FLIR_TRACKER_H

#include <opencv2/core/core.hpp>
#include "Palettes.h"

class FlirTracker
{
public:
    typedef enum _track_res
    {
       TRK_RES_ERROR    = 0,
       TRK_RES_NONE     = 1,
       TRK_RES_FOUND    = 2
    } TrackRes;
     
    typedef enum _track_mode
    {            
        TRK_AVOID   = 0,
        TRK_FOLLOW  = 1
    } TrackMode;

    FlirTracker(TrackMode trkMode, uint16_t minThresh, uint16_t maxThresh, 
        uint8_t rowMin=40, uint8_t rowMax=80);
    ~FlirTracker();
    
    void setNewThresh(uint16_t minThresh, uint16_t maxThresh);
    
    TrackRes setNewFrame( cv::Mat frame16, uint16_t min, uint16_t max  ); //! Set new 16 bit frame and performs tracking step on it
    cv::Mat getResFrameRGB(); //! Return the result of the tracking as RGB frame
    
    static cv::Mat normalizeFrame( const cv::Mat& frame16, uint16_t min, uint16_t max );
    
    void nextPalette();
    
    void setMode( TrackMode newMode ){mTrkMode=newMode;}
        
protected:
    TrackRes doTrackStep();
        
private:   
    TrackMode mTrkMode;
    
    uint16_t mMinThresh;
    uint16_t mMaxThresh;
    
    uint16_t mFrameMin;
    uint16_t mFrameMax;
    
    cv::Mat mFrame16;
    cv::Mat mRes16;
    cv::Mat mResRGB;
    cv::Mat mResMask;
    
    uint8_t mPaletteIdx;
    
    uint8_t mRowMin;
    uint8_t mRowMax;
    
    cv::Point mTargetPos;
    cv::Mat mTargHist;
};

#endif
