#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <ncurses.h>


#include "Lepton3.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "videoEncoder.h"
#include "flir_tracker.h"

#include "stopwatch.hpp"

#include <gst/gst.h>

using namespace std;

static bool quit = false;

void close_handler(int s);

void print_info();

int main( int argc, char *argv[] )
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
    uint32_t raw_ip_port;
    uint32_t res_ip_port;
    std::string iface;

    if( argc != 6 && argc != 7  )
    {
        print_info();

        return EXIT_FAILURE;
    }
    else
    {
        track_mode = argv[1];
        ip_addr = argv[2];

        string raw_port = argv[3];

        try
        {
            raw_ip_port = std::stoul(raw_port);
        }
        catch( const std::invalid_argument& ia )
        {
            print_info();

            cerr << endl <<  "Invalid argument for 'raw_port': " << ia.what() << endl;

            return EXIT_FAILURE;
        }

        string res_port = argv[4];

        try
        {
            res_ip_port = std::stoul(res_port);
        }
        catch( const std::invalid_argument& ia )
        {
            print_info();

            cerr << endl <<  "Invalid argument for 'res_port': " << ia.what() << endl;

            return EXIT_FAILURE;
        }

        iface = argv[5];

        if( argc==7 )
        {
            int dbg = atoi(argv[6]);

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

    // >>>>> Create GStreamer encoder for raw stream
    videoEncoder* gstEncoderRaw = NULL;

    gstEncoderRaw = videoEncoder::Create( 160, 120,
                                       iface,
                                       ip_addr,
                                       raw_ip_port,
                                       1024,
                                       "gst_raw_therm");

    if( !gstEncoderRaw)
    {
        cerr << "Cannot create Encoder for Raw Video Stream" << endl;

        return EXIT_FAILURE;
    }

    gstEncoderRaw->Open();
    // <<<<< Create GStreamer encoder for raw stream

    // >>>>> Create GStreamer encoder for res stream
    videoEncoder* gstEncoderRes = NULL;

    gstEncoderRes = videoEncoder::Create( 160, 120,
                                       iface,
                                       ip_addr,
                                       res_ip_port,
                                       1024,
                                       "gst_res_therm");

    if( !gstEncoderRes)
    {
        cerr << "Cannot create Encoder for Result Video Stream" << endl;

        return EXIT_FAILURE;
    }

    gstEncoderRes->Open();
    // <<<<< Create GStreamer encoder for res stream */

    Lepton3 lepton3( "/dev/spidev1.0", 1, deb_lvl );
    
    /*/ >>>>> Init reboot
    lepton3.rebootCamera();    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // <<<<< Init reboot */
    
    if( lepton3.enableAgc( false ) < 0)
    {
        cerr << "Failed to disable AGC" << endl;
        return EXIT_FAILURE;
    }

    if( lepton3.enableRgbOutput( false ) < 0 )
    {
        cerr << "Failed to disable RGB output" << endl;
        return EXIT_FAILURE;
    }

    // >>>>> Life detection thresholds
    if( lepton3.setGainMode( LEP_SYS_GAIN_MODE_HIGH ) < 0 )
    {
        cerr << "Failed to set Gain mode" << endl;
        return EXIT_FAILURE;
    }

    // Hypothesis: sensor is linear.
    // If the range of the sensor is 0°-150° in High Gain mode, we can calculate the threasholds
    // for "life temperature" between 30.0°C and 37.0°C
    uint16_t min_thresh = 3334; // [0°C-150°C] 3334*0.009 = 30.0°C 
    uint16_t max_thresh = 4112; // [0°C-150°C] 4112*0.009 = 37.0°C 
    
    if( lepton3.enableRadiometry( true ) < 0)
    {
        cerr << "Failed to enable radiometry!" << endl;
        return EXIT_FAILURE;
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
        print_info();

        cerr << endl <<  "Invalid argument for 'trk_mode'" << endl;

        return EXIT_FAILURE;
    }

    FlirTracker tracker( trkMode, min_thresh, max_thresh );
    // <<<<< Tracker    
    
    // >>>>> Get char without enter
    initscr();
    timeout(0);
    cbreak(); /* as per recommend Thomas Dickey */
    noecho(); /* as per recommend Thomas Dickey */    
    // <<<<< Get char without enter

    lepton3.start();

    uint64_t frameIdx=0;

    StopWatch stpWtc;

    stpWtc.tic();

    double period_mean=0.0;
    double freq = 0.0;
    uint16_t min;
    uint16_t max;
    
    /*if( lepton3.doFFC() != LEP_OK )
    {
        return EXIT_FAILURE;
    }*/
    
    while(!quit)
    {        
        uint8_t w,h;

        const uint16_t* data16 = lepton3.getLastFrame16( w, h, &min, &max );

        if( data16 )
        {
            double period_usec = stpWtc.toc();
            stpWtc.tic();
	    period_mean = 0.9*period_mean + 0.1*period_usec;

            freq = (1000.*1000.)/period_mean;

            cv::Mat frame16( h, w, CV_16UC1 );

            cv::Mat frameRes;

            memcpy( frame16.data, data16, w*h*sizeof(uint16_t) );

            // >>>>> Tracking
            if( tracker.setNewFrame( frame16, min, max ) != FlirTracker::TRK_RES_ERROR )
            {
                if( deb_lvl>=Lepton3::DBG_INFO  )
                {
                    cout << "*** Error performing tracking step on frame #" << frameIdx << endl;
                }

                frameRes = tracker.getResFrameRGB();
            }
            // <<<<< Tracking

            if(gstEncoderRaw)
            {
                cv::Mat frameRGB;

                frameRGB = FlirTracker::normalizeFrame( frame16, min, max );
                cv::cvtColor(frameRGB,frameRGB,CV_GRAY2BGR);

                cv::Mat frameYUV( h+h/2, w, CV_8UC1 );
                cv::cvtColor( frameRGB, frameYUV, cv::COLOR_RGB2YUV_I420 );
                gstEncoderRaw->PushFrame( (uint8_t*)frameYUV.data );
                
                if( deb_lvl>=Lepton3::DBG_FULL  )
                {
                    cout << "Frame Raw pushed" << "\r" << endl;
                }
            }

            if(gstEncoderRes && !frameRes.empty() )
            {
                cv::Mat frameYUV( h+h/2, w, CV_8UC1 );
                cv::cvtColor( frameRes, frameYUV, cv::COLOR_RGB2YUV_I420 );
                gstEncoderRes->PushFrame( (uint8_t*)frameYUV.data );

                if( deb_lvl>=Lepton3::DBG_FULL  )
                {
                    cout << "Frame Res pushed" << "\r" << endl;
                }
            }

            frameIdx++;

            if( deb_lvl>=Lepton3::DBG_INFO  )
            {
                cout << "> Frame period: " << period_mean <<  " usec - FPS: " << freq << "\r" << endl;
            }
            
            
            if( frameIdx == 50 )
            {
                lepton3.doRadFFC();
            }
        }
        
        // >>>>> Keyboard interaction
                
        uint8_t minRow, maxRow;                
        int c = getch();           
        
        switch(c)
        {
        case '1':
            tracker.setMode( FlirTracker::TRK_AVOID );
        break;
        
        case '2':
            tracker.setMode( FlirTracker::TRK_FOLLOW );
        break;   
        
        case '+':
            tracker.getCentralZone( minRow, maxRow ); 
            tracker.setCentralZone( --minRow, ++maxRow );
        break;
        
        case '-':
            tracker.getCentralZone( minRow, maxRow ); 
            tracker.setCentralZone( ++minRow, --maxRow );
        break;
        
	    case 'M':
	    case 'm':
            cout << "> Frame period: " << period_mean <<  " usec - FPS: " << freq << "\r" << endl; 
            cout << "> Frame min: " << min <<  " - Frame max: " << max << "\r" << endl;           
        break;

        case 'Q':
        case 'q':
            quit=true;
        break;

	    case 'R':
        case 'r':
            lepton3.rebootCamera();
        break;

	    case 'I':
        case 'i':
            lepton3.doFFC();
        break;
        
        case 'O':
        case 'o':
            lepton3.doRadFFC();
        break;
		
        case 'W':
        case 'w':
            lepton3.saveParams();
        break;
    
        case 'P':
        case 'p':
            tracker.nextPalette();
        break;                     
        
        case 'G':
        case 'g':
            min_thresh += 100;
            max_thresh += 100;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'B':
        case 'b':
            min_thresh -= 100;
            max_thresh -= 100;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
         
        case 'A':
        case 'a':
            printw("\r");
            min_thresh += 10;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'Z':
        case 'z':
            printw("\r");
            min_thresh -= 10;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'S':
        case 's':
            printw("\r");
            min_thresh += 1;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'X':
        case 'x':
            printw("\r");
            min_thresh -= 1;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'D':
        case 'd':
            printw("\r");
            max_thresh += 10;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'C':
        case 'c':
            printw("\r");
            max_thresh -= 10;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'F':
        case 'f':
            printw("\r");
            max_thresh += 1;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;
        
        case 'V':
        case 'v':
            printw("\r");
            max_thresh -= 1;
            tracker.setNewThresh(min_thresh, max_thresh);
        break;        
        }        
           
        
        // <<<<< Keyboard interaction        
        
        //refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
    }

    lepton3.stop();

    if(gstEncoderRaw)
    {
        delete gstEncoderRaw;
    }

    if(gstEncoderRes)
    {
        delete gstEncoderRes;
    }
    
    // ncurses close
    endwin();
    

    return EXIT_SUCCESS;
}

void close_handler(int s)
{
    if(s==2)
    {
        cout << endl << "Ctrl+C pressed..." << endl;
        quit = true;
    }
}

void print_info()
{
    cout << "Usage: " << endl << "   bbb_life_tracker <trk_mode> <debug_ip_address> <raw_port> <res_port>  <multicast_interface> [debug_level]" << endl << endl;
    cout << "   trk_mode:" << endl;
    cout << "       --avoid [-A] / --follow [-F] -> avoid/follow elements with a temperature compatible to life" << endl;
    cout << "   stream_ip_address -> the IP address of the destination of the video stream" << endl;
    cout << "   raw_port -> the port  of the destination of the raw video stream" << endl;
    cout << "   res_port -> the port  of the destination of the video stream with tracking result" << endl;
    cout << "   multicast_interface -> the network interface to multicast the video stream [use '' for unicast]" << endl;
    cout << "   debug_level [optional]:" << endl;
    cout << "       0 [default] (no debug) - 1 (info debug) - 2 (full debug)" << endl << endl;
}
