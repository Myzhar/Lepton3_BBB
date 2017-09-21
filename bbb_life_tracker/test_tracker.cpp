#include <cstdlib>
#include <iostream>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <flir_tracker.h>

#define ROWS 120
#define COLS 160

using namespace std;

int main( int argc, char* argv[] )
{
    std::srand(std::time(0)); // use current time as seed for random generator

    cv::Mat test16( ROWS,COLS, CV_16UC1 );

    uint16_t max_val = static_cast<uint16_t>(pow(2,14)-1);

    uint16_t minTh = max_val/8;
    uint16_t maxTh = max_val/4;

    for( int r=0; r<ROWS; r++  )
    {
        for( int c=0; c<COLS; c++  )
        {
            double rnd = static_cast<double>(std::rand())/RAND_MAX;

            uint16_t rnd_val;

            if( r<90 && r>30 && c<110 && c>50 )
            {
                rnd_val = static_cast<uint16_t>(minTh+rnd*(maxTh-minTh));

            }
            else
            {
                rnd_val = static_cast<uint16_t>(rnd*max_val);
            }

            test16.at<uint16_t>(r,c) = rnd_val;
        }
    }

    cv::Mat resRGB;

    FlirTracker tracker( FlirTracker::TRK_AVOID, minTh, maxTh );

    if( tracker.setNewFrame( test16, 0, max_val ) != FlirTracker::TRK_RES_ERROR )
    {
        resRGB = tracker.getResFrameRGB();
    }

    cv::Mat test8 = FlirTracker::normalizeFrame( test16, 0, max_val );

    cv::imshow( "Original", test8 );
    cv::imshow( "Result", resRGB );

    cv::waitKey();

    return EXIT_SUCCESS;
}
