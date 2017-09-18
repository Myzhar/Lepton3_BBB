#include "videoEncoder.h"

#include <gst/gstsample.h>

#include <iostream>
#include <sstream> 
#include <algorithm>
#include <time.h>

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

//using namespace std;


//------------------------------------------------------------------------------
static void gst_print_one_tag(const GstTagList * list, const gchar * tag, gpointer user_data)
{
    int i, num;

    num = gst_tag_list_get_tag_size (list, tag);
    for (i = 0; i < num; ++i) {
        const GValue *val;

        /* Note: when looking for specific tags, use the gst_tag_list_get_xyz() API,
     * we only use the GValue approach here because it is more generic */
        val = gst_tag_list_get_value_index (list, tag, i);
        if (G_VALUE_HOLDS_STRING (val)) {
            printf("\t%20s : %s\n", tag, g_value_get_string (val));
        } else if (G_VALUE_HOLDS_UINT (val)) {
            printf("\t%20s : %u\n", tag, g_value_get_uint (val));
        } else if (G_VALUE_HOLDS_DOUBLE (val)) {
            printf("\t%20s : %g\n", tag, g_value_get_double (val));
        } else if (G_VALUE_HOLDS_BOOLEAN (val)) {
            printf("\t%20s : %s\n", tag,
                   (g_value_get_boolean (val)) ? "true" : "false");
        } else if (GST_VALUE_HOLDS_BUFFER (val)) {
            GstBuffer *buf = gst_value_get_buffer (val);
            //guint buffer_size = GST_BUFFER_SIZE(buf); // v0.1
            guint buffer_size = gst_buffer_get_size(buf); // v1.0

            printf("\t%20s : buffer of size %u\n", tag, buffer_size);
        }
        else {
            printf("\t%20s : tag of type '%s'\n", tag, G_VALUE_TYPE_NAME (val));
        }
    }
}

static const char* gst_stream_status_string( GstStreamStatusType status )
{
    switch(status)
    {
    case GST_STREAM_STATUS_TYPE_CREATE:     return "CREATE";
    case GST_STREAM_STATUS_TYPE_ENTER:		return "ENTER";
    case GST_STREAM_STATUS_TYPE_LEAVE:		return "LEAVE";
    case GST_STREAM_STATUS_TYPE_DESTROY:	return "DESTROY";
    case GST_STREAM_STATUS_TYPE_START:		return "START";
    case GST_STREAM_STATUS_TYPE_PAUSE:		return "PAUSE";
    case GST_STREAM_STATUS_TYPE_STOP:		return "STOP";
    default:                                return "UNKNOWN";
    }
}

// gst_message_print
gboolean gst_message_print(GstBus* bus, GstMessage* message, gpointer user_data)
{

    switch (GST_MESSAGE_TYPE (message))
    {
    case GST_MESSAGE_ERROR:
    {
        GError *err = NULL;
        gchar *dbg_info = NULL;

        gst_message_parse_error (message, &err, &dbg_info);
        printf( "gstreamer %s ERROR %s\n", GST_OBJECT_NAME (message->src), err->message);
        printf( "gstreamer Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
        
        g_error_free(err);
        g_free(dbg_info);

        break;
    }
    case GST_MESSAGE_EOS:
    {
        printf( "gstreamer %s recieved EOS signal...\n", GST_OBJECT_NAME(message->src));
        //g_main_loop_quit (app->loop);		// TODO trigger plugin Close() upon error
        break;
    }
    case GST_MESSAGE_STATE_CHANGED:
    {
        GstState old_state, new_state;

        gst_message_parse_state_changed(message, &old_state, &new_state, NULL);

        printf( "gstreamer changed state from %s to %s ==> %s\n",
                gst_element_state_get_name(old_state),
                gst_element_state_get_name(new_state),
                GST_OBJECT_NAME(message->src));
        break;
    }
    case GST_MESSAGE_STREAM_STATUS:
    {
        GstStreamStatusType streamStatus;
        gst_message_parse_stream_status(message, &streamStatus, NULL);

        printf( "gstreamer stream status %s ==> %s\n",
                gst_stream_status_string(streamStatus),
                GST_OBJECT_NAME(message->src));
        break;
    }
    case GST_MESSAGE_TAG:
    {
        GstTagList *tags = NULL;

        gst_message_parse_tag(message, &tags);

#ifdef gst_tag_list_to_string
        gchar* txt = gst_tag_list_to_string(tags);
#else
        gchar txt[] = {"missing gst_tag_list_to_string()"};
#endif

        printf( "gstreamer %s %s\n", GST_OBJECT_NAME(message->src), txt);

        g_free(txt);
        //gst_tag_list_foreach(tags, gst_print_one_tag, NULL);
        gst_tag_list_free(tags);
        break;
    }
    default:
    {
        printf( "gstreamer msg %s ==> %s\n", gst_message_type_get_name(GST_MESSAGE_TYPE(message)), GST_OBJECT_NAME(message->src));
        break;
    }
    }

    return TRUE;
}

std::string strFileExtension( const std::string& path )
{
    std::string ext = path.substr(path.find_last_of(".") + 1);
    transform(ext.begin(), ext.end(), ext.begin(), tolower);
    return ext;
}

void sleep_ms( uint64_t milliseconds )
{
    timespec duration;
    duration.tv_sec  = 0;
    duration.tv_nsec = milliseconds * 1000 * 1000;
    nanosleep(&duration, NULL);
}

//------------------------------------------------------------------------------


// constructor
videoEncoder::videoEncoder(uint32_t width, uint32_t height, std::string multicastIface, uint32_t bitrate , std::string encName )
{	
    mAppSrc         = NULL;
    mBus            = NULL;
    //mBufferCaps    = NULL;
    mPipeline       = NULL;
    mNeedData       = false;
    mInputBuffer    = NULL;
    mOutputPort     = 0;
    mOpened         = false;
    mWidth          = width;
    mHeight         = height;
    mBitRate        = bitrate;

    mMulticastIface = multicastIface;

    mEncName = encName;
}


// destructor	
videoEncoder::~videoEncoder()
{
    Close();

    if( mInputBuffer != NULL )
    {
        gst_buffer_unref(mInputBuffer);
        mInputBuffer = NULL;
    }
}

// Create
videoEncoder* videoEncoder::Create( uint32_t width, uint32_t height, std::string filename, std::string encName )
{
    videoEncoder* v = new videoEncoder(width, height, "", 8000000, encName);

    v->mOutputPath = filename;

    if( !v->init() )
    {
        delete v;
        return NULL;
    }

    return v;
}

videoEncoder* videoEncoder::Create( uint32_t width, uint32_t height, std::string multicastIface, std::string ipAddress, uint32_t port, uint32_t bitrate, std::string encName )
{
    videoEncoder* v = new videoEncoder(width, height, multicastIface, bitrate, encName );

    v->mOutputIP   = ipAddress;
    v->mOutputPort = port;

    if( !v->init() )
    {
        delete v;
        return NULL;
    }

    return v;
}

// onNeed
void videoEncoder::onNeed( GstElement * pipeline, guint size, gpointer user_data )
{
    //printf("gstreamer appsrc requesting data (%u bytes)\n", size);

    if( !user_data )
        return;

    videoEncoder* enc = (videoEncoder*)user_data;
    enc->mNeedData  = true;
}



// onEnough
void videoEncoder::onEnough(GstElement * pipeline, gpointer user_data)
{
    //printf("gstreamer appsrc signalling enough data\n");

    if( !user_data )
        return;

    videoEncoder* enc = (videoEncoder*)user_data;
    enc->mNeedData  = false;
}


// ProcessBuffer
bool videoEncoder::PushFrame( uint8_t* buf )
{
    //msgOut( ">>>>> ProcessFrame" );

    if( !buf )
    {
        msgOut( "***  No buffer ***" );
        return false;
    }

    if( !mNeedData )
    {
        msgOut( "***  No data needed ***" );
        return true;
    }

    // >>>>> GStreamer v1.0


    //    if( !gst_buffer_is_writable(mInputBuffer) )
    //    {
    //        msgOut( " *** Buffer not writable ***" );
    //        //gst_buffer_make_writable(mInputBuffer);
    //        //msgOut( "<<<<< ProcessFrame" );
    //        return false;
    //    }

    mInputBuffer = gst_buffer_make_writable( mInputBuffer );

    if( !mInputBuffer )
    {
        msgOut( " *** Buffer not writable ***" );
        return false;
    }

    GstMapInfo mapInfo;
    if( gst_buffer_map( mInputBuffer, &mapInfo, (GstMapFlags)(GST_MAP_READWRITE)) )
    {
        //mapInfo.data = buffer;

        int buf_size = gst_buffer_get_size(mInputBuffer);

        memcpy( mapInfo.data, buf, buf_size );
        // <<<<< GStreamer v1.0

        mNeedData  = false;

        // queue buffer to gstreamer

        GstFlowReturn ret;
        g_signal_emit_by_name(mAppSrc, "push-buffer", mInputBuffer, &ret);

        if( ret != 0 )
            fprintf(stderr, "gstreamer -- AppSrc pushed buffer (result %u)\r\n", ret);

        // check messages
        while(true)
        {
            GstMessage* msg = gst_bus_pop(mBus);

            if( !msg )
                break;

            gst_message_print(mBus, msg, this);
            gst_message_unref(msg);
        }

        gst_buffer_unmap( mInputBuffer, &mapInfo );
    }
    else
    {
        msgOut( "*** Buffer not ready ***" );
    }

    //msgOut( "<<<<< ProcessFrame" );

    return true;
}

bool videoEncoder::buildCapsStr()
{
    std::ostringstream ss;
    ss << "video/x-raw, format=I420, framerate=9/1, width=" << mWidth << ", height=" << mHeight << " "; // v1.0

    mCapsStr = ss.str();

    return true;
}


// buildLaunchStr
bool videoEncoder::buildLaunchStr()
{
    const size_t fileLen = mOutputPath.size();
    const size_t ipLen   = mOutputIP.size();

    std::ostringstream ss;
    ss << "appsrc name=mysource ! ";
    ss << mCapsStr.c_str();
    ss << " ! queue";
    ss << " ! omxh264enc ";
    ss << "bitrate=" << mBitRate << " "; // H264 encoder bitrate

    ss << "insert-sps-pps=true ";        // insert-sps-pps=true needed to start client after server!!!

    //ss << "control-rate=2 ";
    //ss << "low-latency=true ";

    ss << "quant-i-frames=5 ";
    ss << "quant-p-frames=2 ";
    ss << "quant-b-frames=2 ";
    ss << "iframeinterval=1 ";
    ss << "quality-level=2 ";
    ss << "! ";


    if( fileLen > 0 && ipLen > 0 )
        ss << "tee name=t ! ";

    if( fileLen > 0 )
    {
        std::string ext = strFileExtension(mOutputPath);

        if( strcasecmp(ext.c_str(), "mkv") == 0 )
        {
            //ss << "matroskamux ! queue ! ";
            ss << "matroskamux ! ";
        }
        else if( strcasecmp(ext.c_str(), "h264") != 0 )
        {
            printf( "gstreamer invalid output extension %s\n", ext.c_str());
            return false;
        }

        ss << "filesink location=" << mOutputPath;

        if( ipLen > 0 )
            ss << " t. ! ";	// begin the second tee
    }

    if( ipLen > 0 )
    {
        ss << "rtph264pay ";
        ss << "pt=96 ";
        ss << "config-interval=1 ";
        ss << "! ";
        ss << "queue ! ";
        ss << "udpsink host=";
        ss << mOutputIP;

        if( mOutputPort != 0 )
            ss << " port=" << mOutputPort;

        if( !mMulticastIface.empty() )
        {
            ss << " multicast-iface=";
            ss << mMulticastIface;
            ss << " force-ipv4=true";
            ss << " auto-multicast=true";
            ss << " ttl-mc=128"; // Time To Live per fare uscire il multicast dalla sottorete!!!
        }
        ss << " sync=false async=false";//*/

        /*//ss << " h264parse !";
        ss << " mpegtsmux !";
        ss << " rtpmp2tpay !";
        ss << " udpsink host=";
        ss << mOutputIP << " ";

        if( mOutputPort != 0 )
            ss << "port=" << mOutputPort;//*/

    }

    // >>>>>

    mLaunchStr = ss.str();

    std::ostringstream ssDeb;
    ssDeb << "gstreamer encoder pipeline string: " << std::endl << mLaunchStr << std::endl << std::endl;

    std::string msg = ssDeb.str();
    msgOut( msg );

    //std::cout << std::endl << "GStreamer Pipeline: " << msg << std::endl;

    return true;
}

// init
bool videoEncoder::init()
{
    if( mWidth == 0 || mHeight == 0 )
    {
        printf( "gstreamer -- invalid width/height (%u x %u)\n", mWidth, mHeight);
        return false;
    }

    // build pipeline string
    if( buildCapsStr() )
    {
        if( !buildLaunchStr() )
        {
            printf( "gstreamer failed to build pipeline string\n");
            return false;
        }
    }
    else
    {
        printf( "gstreamer failed to build pipeline caps string\n");
        return false;
    }

    // launch pipeline
    GError* err = NULL;
    mPipeline   = gst_parse_launch(mLaunchStr.c_str(), &err);

    if( err != NULL )
    {
        printf( "gstreamer failed to create pipeline\n\r");
        printf( "   (%s)\n\r", err->message);
        g_error_free(err);
        return false;
    }

    GstPipeline* pipeline = GST_PIPELINE(mPipeline);

    if( !pipeline )
    {
        printf( "gstreamer failed to cast GstElement into GstPipeline\n");
        return false;
    }

    // retrieve pipeline bus
    mBus = gst_pipeline_get_bus(pipeline);

    if( !mBus )
    {
        printf( "gstreamer failed to retrieve GstBus from pipeline\n");
        return false;
    }

    // get the appsrc
    GstElement* appsrcElement = gst_bin_get_by_name(GST_BIN(pipeline), "mysource");
    GstAppSrc* appsrc = GST_APP_SRC(appsrcElement);

    if( !appsrcElement || !appsrc )
    {
        printf( "gstreamer failed to retrieve AppSrc element from pipeline\n");
        return false;
    }

    mAppSrc = appsrcElement;

    g_signal_connect(appsrcElement, "need-data", G_CALLBACK(onNeed), this);
    g_signal_connect(appsrcElement, "enough-data", G_CALLBACK(onEnough), this);


    mInputBuffer = gst_buffer_new_allocate(NULL, (mWidth * mHeight * 12) / 8, NULL ); // v1.0

    gst_app_src_set_stream_type(appsrc, GST_APP_STREAM_TYPE_STREAM);
    g_object_set(G_OBJECT(mAppSrc), "is-live", TRUE, NULL);
    g_object_set(G_OBJECT(mAppSrc), "do-timestamp", TRUE, NULL);
    g_object_set(G_OBJECT(mAppSrc), "format", GST_FORMAT_TIME, NULL); // v1.0 to avoid "gst_segment_to_running_time: assertion 'segment->format == format' failed" error

    return true;
}

// Open
bool videoEncoder::Open()
{
    if( mOpened )
        return true;

    printf( "gstreamer transitioning pipeline to GST_STATE_PLAYING\n");
    const GstStateChangeReturn result = gst_element_set_state(mPipeline, GST_STATE_PLAYING);

    if( result == GST_STATE_CHANGE_ASYNC )
    {
#if 0
        GstMessage* asyncMsg = gst_bus_timed_pop_filtered(mBus, 5 * GST_SECOND,
                                                          (GstMessageType)(GST_MESSAGE_ASYNC_DONE|GST_MESSAGE_ERROR));
        if( asyncMsg != NULL )
        {
            gst_message_print(mBus, asyncMsg, this);
            gst_message_unref(asyncMsg);
        }
        else
            printf( "gstreamer NULL message after transitioning pipeline to PLAYING...\n");
#endif
    }
    else if( result != GST_STATE_CHANGE_SUCCESS )
    {
        printf( "gstreamer failed to set pipeline state to PLAYING (error %u)\n", result);
        return false;
    }

    mOpened = true;

    return true;
}

// Close
bool videoEncoder::Close()
{
    if( !mOpened )
        return true;

    // send EOS
    mNeedData = false;

    printf( "gstreamer sending encoder EOS\n");
    GstFlowReturn eos_result = gst_app_src_end_of_stream(GST_APP_SRC(mAppSrc));

    if( eos_result != 0 )
        printf( "gstreamer failed sending appsrc EOS (result %u)\n", eos_result);

    sleep_ms(250);

    // stop pipeline
    printf( "gstreamer transitioning pipeline to GST_STATE_NULL\n");

    const GstStateChangeReturn result = gst_element_set_state(mPipeline, GST_STATE_NULL);

    if( result != GST_STATE_CHANGE_SUCCESS )
        printf( "gstreamer failed to set pipeline state to PLAYING (error %u)\n", result);

    sleep_ms(250);
    mOpened = false;
    return true;
}

void videoEncoder::msgOut( const std::string message )
{
    std::ostringstream ss;
    ss << "[" << mEncName << "] -> " << message << std::endl;

    std::cout << ss.str();
}
