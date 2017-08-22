#include "lepton3_grabber.hpp"
#include <iostream>

using namespace std;
using msec = std::chrono::milliseconds;

l3_grabber::l3_grabber(std::string spi_port)
	: mThread()
	, mFrameW(FRAME_W)
	, mFrameH(FRAME_H)
{
	mStop = false;
}

l3_grabber::~l3_grabber()
{
	mStop = true;
        
    if(mThread.joinable()) 
    {
    	mThread.join();
    }
}

bool l3_grabber::start()
{
	mThread = std::thread( &l3_grabber::acquire_thread_func, this );
}

const char* l3_grabber::getLastFrame(int* outW, int* outH)
{
	
	if (mFrameMutex.try_lock_for(msec(500))) 
	{	
		if(outW)
		{
			*outW = mFrameW;
		}
	
		if(outH)
		{
			*outH = mFrameH;
		}
	
		mFrameMutex.unlock();
	
		return mLastFrame;
	}
	
	if(outW)
	{
		*outW = -1;
	}

	if(outH)
	{
		*outH = -1;
	}
	
	return NULL;
}

void l3_grabber::acquire_thread_func()
{
	cout << "Acquire thread started ..." << endl; 
	
	while(1)
	{
		if( mStop )
		{
			cout << "... Acquire thread stopped ..." << endl; 
			break;
		}
	}
	
	cout << "... acquire thread finished" << endl; 
}
