#include <cstdlib>
#include <iostream>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <flir_tracker.h>
#include "Palettes.h"

#define ROWS 120
#define COLS 160

using namespace std;

int main( int argc, char* argv[] )
{
    cv::Mat resRGB;

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

    FlirTracker tracker( FlirTracker::TRK_AVOID, minTh, maxTh );

    if( tracker.setNewFrame( test16, 0, max_val ) != FlirTracker::TRK_RES_ERROR )
    {
        resRGB = tracker.getResFrameRGB();
    }

    cv::Mat test8 = FlirTracker::normalizeFrame( test16, 0, max_val );

    for( int i=0; i<PALETTES_COUNT; i++ )
    {
        const uint8_t* lut = palettes[i];

        int scaleC = 100;
        cv::Mat scale( 256,scaleC, CV_8UC3, cv::Scalar(0,0,0) );
        for( int r=0; r<256; r++ )
        {
            for( int c=0; c<scaleC; c++ )
            {
                int idx = 3*((255-r)*scaleC+c);
                scale.data[idx+2] = lut[r*3+0];
                scale.data[idx+1] = lut[r*3+1];
                scale.data[idx+0] = lut[r*3+2];
            }
        }
        cv::string label = "#";
        label += std::to_string( i );
        cv::imshow( label , scale );
    }



    cv::imshow( "Original", test8 );
    cv::imshow( "Result", resRGB );

    cv::waitKey();

    return EXIT_SUCCESS;
}
