#include "flir_tracker.h"


FlirTracker::FlirTracker(TrackMode trkMode, uint16_t minThresh, uint16_t maxThresh)
    : mTrkMode(trkMode)
    , mMinThresh(minThresh)
    , mMaxThresh(maxThresh)
{

}

FlirTracker::~FlirTracker()
{

}

bool FlirTracker::setNewFrame( cv::Mat frame16 )
{
    mFrame16 = frame16;

    return doTrackStep();
}

bool FlirTracker::doTrackStep()
{

}
