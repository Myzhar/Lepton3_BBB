#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <chrono>
#include <thread>

#include "Lepton3.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "videoEncoder.h"
#include "flir_tracker.h"

#include "stopwatch.hpp"

#include <gst/gst.h>

using namespace std;

static bool close = false;

void close_handler(int s)
{
    if(s==2)
    {
        cout << endl << "Ctrl+C pressed..." << endl;
        close = true;
    }
}

void print_info()
{
    cout << "Usage: " << endl << "\t bbb_life_tracker <trk_mode> <debug_ip_address> <debug_port> <multicast_interface> [debug_level]" << endl << endl;
    cout << "\ttrk_mode:" << endl;
    cout << "\t\t --avoid [-A] / --follow [-F] -> avoid/follow elements with a temperature compatible to life" << endl;
    cout << "\tdebug_ip_address -> the IP address of the destination of the video stream" << endl;
    cout << "\tdebug_port -> the port  of the destination of the video stream" << endl;
    cout << "\tmulticast_interface -> the network interface to multicast the video stream [use \"\" for unicast]" << endl;
    cout << "\tdebug_level [optional]:" << endl;
    cout << "\t\t 0 [default] (no debug) - 1 (info debug) - 2 (full debug)" << endl << endl;
}

int main( int   argc, char *argv[] )
{
    gst_init( &argc, &argv );

    cout << "Life Tracker demo for Lepton3 on BeagleBoard Blue" << endl;

    // >>>>> Enable Ctrl+C
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = close_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // <<<<< Enable Ctrl+C

    Lepton3::DebugLvl deb_lvl = Lepton3::DBG_NONE;

    std::string track_mode;
    std::string ip_addr;
    uint32_t ip_port;
    std::string iface;

    if( argc != 5 || argc != 6  )
    {
        print_info();

        return EXIT_FAILURE;
    }
    else
    {
        track_mode = argv[1];
        ip_addr = argv[2];
        string port = argv[3];

        try
        {
            ip_port = std::stoul(port);
        }
        catch( const std::invalid_argument& ia )
        {
            print_info();

            cerr << endl <<  "Invalid argument for 'debug_port': " << ia.what() << endl;

            return EXIT_FAILURE;
        }


        iface = argv[4];

        if( argc==6 )
        {
            int dbg = atoi(argv[5]);

            switch( dbg )
            {
            case 1:
                deb_lvl = Lepton3::DBG_INFO;
                break;

            case 2:
                deb_lvl = Lepton3::DBG_FULL;
                break;

            default:
            case 0:
                deb_lvl = Lepton3::DBG_NONE;
                break;
            }
        }
    }

    // >>>>> Create GStreamer encoder
    videoEncoder* gstEncoder = NULL;

    gstEncoder = videoEncoder::Create( 160, 120,
                                       iface,
                                       ip_addr,
                                       ip_port,
                                       1024,
                                       "gst_encoder");

    gstEncoder->Open();
    // <<<<< Create GStreamer encoder

    Lepton3 lepton3( "/dev/spidev1.0", 1, deb_lvl );
    
    if( lepton3.enableRadiometry( true ) < 0)
    {
        cerr << "Failed to enable radiometry!" << endl;
        return EXIT_FAILURE;
    }

    if( lepton3.enableAgc( FALSE ) < 0)
    {
        cerr << "Failed to disable AGC" << endl;
    }

    if( lepton3.enableRgbOutput( false ) < 0 )
    {
        cerr << "Failed to disable RGB output" << endl;
    }

    // >>>>> Life detection thresholds
    // thresholds settings according to Lepton3 gain mode
    LEP_SYS_GAIN_MODE_E gainMode;
    if( lepton3.getGainMode( gainMode ) == LEP_OK )
    {
        string str = (gainMode==LEP_SYS_GAIN_MODE_HIGH)?string("High"):((gainMode==LEP_SYS_GAIN_MODE_LOW)?string("Low"):string("Auto"));
        cout << " * Gain mode: " << str << endl;
    }

    if( gainMode==LEP_SYS_GAIN_MODE_AUTO ) // we want a fixed gain mode!
    {
        if( lepton3.setGainMode( LEP_SYS_GAIN_MODE_HIGH ) < 0 )
        {
            cerr << "Failed to set Gain mode" << endl;
        }
    }

    uint16_t min_thresh;
    uint16_t max_thresh;

    if( gainMode == LEP_SYS_GAIN_MODE_HIGH )
    {
        min_thresh = 300; // TODO evaluate these thresholds
        max_thresh = 600; // TODO evaluate these thresholds
    }
    else
    {
        min_thresh = 300; // TODO evaluate these thresholds
        max_thresh = 600; // TODO evaluate these thresholds
    }
    // <<<<< Life detection thresholds

    // >>>>> Tracker
    FlirTracker::TrackMode trkMode;
    if( track_mode.compare( "--follow") == 0 ||
        track_mode.compare( "-F") ==0 )
    {
        trkMode = FlirTracker::TRK_FOLLOW;
    }
    else if( track_mode.compare( "--avoid") == 0 ||
             track_mode.compare( "-A") == 0 )
    {
        trkMode = FlirTracker::TRK_AVOID;
    }
    else
    {
        lepton3.stop();

        print_info();

        cerr << endl <<  "Invalid argument for 'trk_mode'" << endl;

        return EXIT_FAILURE;
    }

    FlirTracker tracker( trkMode, min_thresh, max_thresh );
    // <<<<< Tracker

    lepton3.start();

    uint64_t frameIdx=0;

    StopWatch stpWtc;

    stpWtc.tic();

    while(!close)
    {
        uint16_t min;
        uint16_t max;
        uint8_t w,h;

        const uint16_t* data16 = lepton3.getLastFrame16( w, h, &min, &max );

        if( data16 )
        {
            double period_usec = stpWtc.toc();
            stpWtc.tic();

            double freq = (1000.*1000.)/period_usec;

            cv::Mat frame16( h, w, CV_16UC1 );
            cv::Mat frameRGB( h, w, CV_8UC3 );

            memcpy( frame16.data, data16, w*h*sizeof(uint16_t) );

            // >>>>> Tracking
            if( !tracker.setNewFrame( frame16 ) )
            {
                if( deb_lvl>=Lepton3::DBG_INFO  )
                {
                    cout << "*** Error performing tracking step on frame #" << frameIdx << endl;
                }
            }
            // <<<<< Tracking

            // >>>>> Rescaling/Normalization to 8bit to visualize data
            double diff = static_cast<double>(max - min); // Image range
            double scale = 255./diff; // Scale factor

            frame16 -= min; // Bias
            frame16 *= scale; // Rescale data

            cv::Mat frame8;
            frame16.convertTo( frame8, CV_8UC1 );
            // <<<<< Rescaling/Normalization to 8bit to visualize data

            cv::cvtColor( frame8,frameRGB, CV_GRAY2RGB );

            if(gstEncoder)
            {
                cv::Mat frameYUV( h+h/2, w, CV_8UC1 );
                cv::cvtColor( frameRGB, frameYUV, cv::COLOR_RGB2YUV_I420 );
                gstEncoder->PushFrame( (uint8_t*)frameYUV.data );
                
                if( deb_lvl>=Lepton3::DBG_FULL  )
                {
                    cout << "Frame pushed" << endl;
                }
            }

            frameIdx++;

            if( deb_lvl>=Lepton3::DBG_INFO  )
            {
                cout << "> Frame period: " << period_usec <<  " usec - FPS: " << freq << endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(110));
    }

    lepton3.stop();

    if(gstEncoder)
    {
        delete gstEncoder;
    }


    return EXIT_SUCCESS;
}
