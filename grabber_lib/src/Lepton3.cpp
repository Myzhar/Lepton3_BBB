#include "Lepton3.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_RAD.h"

#include <bitset>

#define KELVIN (-273.15f)

#define FRAME_W 160
#define FRAME_H 120

using namespace std;

Lepton3::Lepton3(std::string spiDevice, uint16_t cciPort, DebugLvl dbgLvl )
    : mThread()
    , mSpiRawFrameBuf(NULL)
{
    // >>>>> VoSPI
    mSpiDevice = spiDevice;
    mSpiFd = -1;

    mSpiMode = SPI_MODE_3; // CPOL=1 (Clock Idle high level), 
                           // CPHA=1 (SDO transmit/change edge idle to active)
    mSpiBits = 8;
    mSpiSpeed = 32500000; // Max available SPI speed (according to Lepton3 datasheet)

    mPacketCount  = 60;  // default no Telemetry
    mPacketSize   = 164; // default 14 bit raw data
    mSegmentCount = 4;   // 4 segments for each unique frame

    mSegmentFreq = 106.0f; // According to datasheet each segment is ready at 106 Hz
    
    mSegmSize = mPacketCount*mPacketSize;
    mSpiRawFrameBufSize = mSegmSize*mSegmentCount;

    mSpiRawFrameBuf = new uint8_t[mSpiRawFrameBufSize];
    
    mDataFrameBuf = new uint16_t[FRAME_W*FRAME_H];

    mSpiTR.tx_buf = (unsigned long)NULL;
    mSpiTR.delay_usecs = 50;
    mSpiTR.speed_hz = mSpiSpeed;
    mSpiTR.bits_per_word = mSpiBits;
    mSpiTR.cs_change = 0;
    mSpiTR.tx_nbits = 0;
    mSpiTR.rx_nbits = 0;
    mSpiTR.pad = 0;
    // <<<<< VoSPI

    // >>>>> CCI
    mCciPort = cciPort;
    // <<<<< CCI

    mDebugLvl = dbgLvl;

    if( mDebugLvl>=DBG_INFO )
        cout << "Debug level: " << mDebugLvl << endl;

    mStop = false;
    mDataValid = false;
}

Lepton3::~Lepton3()
{
    stop();

    if(mSpiRawFrameBuf)
        delete [] mSpiRawFrameBuf;
        
    if(mDataFrameBuf)
        delete [] mDataFrameBuf;
}

bool Lepton3::start()
{
    mThread = std::thread( &Lepton3::thread_func, this );
}

void Lepton3::stop()
{
    mStop = true;

    if(mThread.joinable())
    {
        mThread.join();
    }
}

bool Lepton3::SpiOpenPort( )
{
    int status_value = -1;

    if( mDebugLvl>=DBG_INFO )
        cout << "Opening SPI device: " << mSpiDevice.c_str() << endl;

    mSpiFd = open(mSpiDevice.c_str(), O_RDWR);

    if(mSpiFd < 0)
    {
        cerr << "Error - Could not open SPI device: " << mSpiDevice.c_str() << endl;
        return false;
    }

    status_value = ioctl(mSpiFd, SPI_IOC_WR_MODE, &mSpiMode);
    if(status_value < 0)
    {
        cerr << "Could not set SPIMode (WR)...ioctl fail" << endl;
        return false;
    }

    status_value = ioctl(mSpiFd, SPI_IOC_RD_MODE, &mSpiMode);
    if(status_value < 0)
    {
        cerr << "Could not set SPIMode (RD)...ioctl fail" << endl;
        return -1;
    }

    if( mDebugLvl>=DBG_INFO )
        cout << "SPI mode: " << (int)mSpiMode << endl;

    status_value = ioctl(mSpiFd, SPI_IOC_WR_BITS_PER_WORD, &mSpiBits);
    if(status_value < 0)
    {
        cerr << "Could not set SPI bitsPerWord (WR)...ioctl fail" << endl;
        return false;
    }

    status_value = ioctl(mSpiFd, SPI_IOC_RD_BITS_PER_WORD, &mSpiBits);
    if(status_value < 0)
    {
        cerr << "Could not set SPI bitsPerWord(RD)...ioctl fail" << endl;
        return false;
    }

    if( mDebugLvl>=DBG_INFO )
        cout << "SPI bits per word: " << (int)mSpiBits << endl;

    status_value = ioctl(mSpiFd, SPI_IOC_WR_MAX_SPEED_HZ, &mSpiSpeed);
    if(status_value < 0)
    {
        cerr << "Could not set SPI speed (WR)...ioctl fail" << endl;
        return false;
    }

    status_value = ioctl(mSpiFd, SPI_IOC_RD_MAX_SPEED_HZ, &mSpiSpeed);
    if(status_value < 0)
    {
        cerr << "Could not set SPI speed (RD)...ioctl fail" << endl;
        return false;
    }

    if( mDebugLvl>=DBG_INFO )
        cout << "SPI max speed: " << (int)mSpiSpeed << endl;

    return true;
}

void Lepton3::SpiClosePort()
{
    if( mSpiFd<0 )
        return;

    int status_value = close(mSpiFd);
    if(status_value < 0)
    {
        cerr << "Error closing SPI device [" << mSpiFd << "] " << mSpiDevice;
    }
}

int Lepton3::SpiReadSegment()
{
    if( mSpiFd<0 )
    {
        if( mDebugLvl>=DBG_FULL )
        {
            cout << "SPI device not open. Trying to open it..." << endl;
        }
        if( !SpiOpenPort() )
            return -1;
    }

    /*********************************************************************************************
    1) Calculate the address of the segment buffer in the "full frame" buffer [mSpiFrameBuf]
    *********************************************************************************************/
    uint8_t* segmentAddr = mSpiRawFrameBuf+(mCurrSegm*mSegmSize);
    
    /*********************************************************************************************
    2) Wait for the first valid packet
       [Packet Header (16 bit) not equal to xFxx and Packet ID equal to 0]
    *********************************************************************************************/

    // >>>>> Wait first valid packet
    mSpiTR.cs_change = 0;
    mSpiTR.rx_buf = (unsigned long)(segmentAddr); // First Packet has been read above
    mSpiTR.len = mPacketSize;
    while(1)
    {
        if( mStop )
        {
            return -1;
        }

        int ret = ioctl( mSpiFd, SPI_IOC_MESSAGE(1), &mSpiTR );
        if (ret == 1)
        {
            cerr << "Error reading full segment from SPI" << endl;
            return -1;
        }

        if( (segmentAddr[0] & 0x0f) == 0x0f ) // Packet not valid
            continue;

        if( segmentAddr[1] == 0 ) // First valid packet
            break;
    }
    // <<<<< Wait first valid packet */

    /*********************************************************************************************
    // 3) Read the full segment
          Note: the packet #0 has been read at step 2, so the number of packets to be read must
                be decreased by a packet size and buffer address must be shifted of a packet size
    *********************************************************************************************/

    // >>>>> Segment reading
    mSpiTR.rx_buf = (unsigned long)(segmentAddr+mPacketSize); // First Packet has been read above
    mSpiTR.len = mSegmSize-mPacketSize;
    mSpiTR.cs_change = 0; // /CS asserted after "ioctl"
    
    int ret = ioctl( mSpiFd, SPI_IOC_MESSAGE(1), &mSpiTR );
    if (ret == 1)
    {
        cerr << "Error reading full segment from SPI" << endl;
        return -1;
    }
    // <<<<< Segment reading

    /*********************************************************************************************
    // 4) Get the Segment ID from packet #20 (21th packet)
    *********************************************************************************************/

    // >>>>> Segment ID
    // Segment ID is written in the 21th Packet int the bit 1-3 of the first byte 
    // (the first bit is always 0)
    // Packet number is written in the bit 4-7 of the first byte

    uint8_t pktNumber = segmentAddr[20*mPacketSize+1];
    
    if( mDebugLvl>=DBG_FULL )
    {
        cout << "{" << (int)pktNumber << "} ";
    }
    
    if( pktNumber!=20 )
    {
        if( mDebugLvl>=DBG_INFO )
        {
            cout << "Wrong Packet ID for TTT in segment" << endl;
        }

        return -1;
    }

    int segmentID = (segmentAddr[20*mPacketSize] & 0x70) >> 4;
    // <<<<< Segment ID

    return segmentID;
}

void Lepton3::thread_func()
{
    if( mDebugLvl>=DBG_INFO )
        cout << "Grabber thread started ..." << endl;

    mStop = false;

    int ret = 0;

    if( !SpiOpenPort() )
    {
        cerr << "Grabber thread stopped on starting for SPI error" << endl;
        return;
    }

    if( mDebugLvl>=DBG_FULL )
        cout << "SPI fd: " << mSpiFd << endl;

    int notValidCount = 0;
    mCurrSegm = 0;

    mThreadWatch.tic();

    StopWatch testTime1, testTime2;

    mDataValid = false;

    while(true)
    {
        // >>>>> Timing info
        double threadPeriod = mThreadWatch.toc(); // Get thread by thread time
        mThreadWatch.tic();

        int usecAvail = (int)((1/mSegmentFreq)*1000*1000)-threadPeriod;

        if( mDebugLvl>=DBG_FULL )
        {
            cout << endl << "Thread period: " << threadPeriod << " usec - VoSPI Available: " 
                << usecAvail << " usec" << endl;
        }

        if( mDebugLvl>=DBG_INFO )
        {
            cout << "VoSPI segment acquire freq: " << (1000.0*1000.0)/threadPeriod 
                << " hz" << endl;
        }
        // <<<<< Timing info

        // >>>>> Acquire single segment
        if( mDebugLvl>=DBG_FULL )
        {
            testTime1.tic();
        }

        int segment = SpiReadSegment();

        if( mDebugLvl>=DBG_FULL )
        {
            double elapsed = testTime1.toc();
            cout << "VoSPI segment read time " << elapsed << " usec" << endl;
        }
        // <<<<< Acquire single segment

        // >>>>> Segment check
        if( mDebugLvl>=DBG_FULL )
        {
            testTime1.tic();
        }

        if( segment!=-1 )
        {
            if( mDebugLvl>=DBG_FULL )
            {
                cout << "Retrieved segment: " << segment;
            }

            if( segment != 0 )
            {
                if( mDebugLvl>=DBG_FULL )
                {
                    cout << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
                }
                
                notValidCount=0;

                if( segment==(mCurrSegm+1) )
                {
                    mCurrSegm=segment;
                }

                if(mCurrSegm==4)
                {
                    // Start a new frame
                    mCurrSegm = 0;

                    // FRAME COMPLETE

                    if( mDebugLvl>=DBG_FULL )
                    {
                        cout << endl 
                            << "************************ FRAME COMPLETE " \
                                "************************" << endl;
                    }

                    // >>>>> RAW to 16bit data conversion
                    if( mDebugLvl>=DBG_FULL )
                    {
                        testTime2.tic();
                    }

                    raw2data();

                    if( mDebugLvl>=DBG_FULL )
                    {
                        double elapsed = testTime2.toc();
                        cout << "VoSPI frame conversion time " << elapsed << " usec" << endl;
                    }

                    mDataValid = true;
                    // <<<<< RAW to 16bit data conversion
                }                
            }
            else
            {
                // Frame abort
                // Start a new frame
                mCurrSegm = 0;

                notValidCount++;
            }
            
            if( mDebugLvl>=DBG_FULL )
            {
                cout << endl;
            }
        }
        else
        {
            // Frame abort
            mCurrSegm = 0;

            notValidCount++;
        }

        if( mDebugLvl>=DBG_FULL )
        {
            double elapsed = testTime1.toc();
            cout << "VoSPI segment check time " << elapsed << " usec" << endl;
        }
        // <<<<< Segment check
        
        // According to datasheet, after 4 valid segments (ID 1,2,3,4) we should 
        // read 8 not valid segments (ID 0)
        // If the number of not valid segments is higher than 8 we need to resync 
        // the host with the device
        if( notValidCount>=10 )
        {
            resync();
            
            notValidCount=0;
        }

        if( mStop )
        {
            if( mDebugLvl>=DBG_INFO )
                cout << "... grabber thread stopped ..." << endl;

            break;
        }
    }

    //finally, close SPI port just bcuz
    SpiClosePort();

    if( mDebugLvl>=DBG_INFO )
        cout << "... grabber thread finished" << endl;
}

void Lepton3::resync()
{
    if( mDebugLvl>=DBG_INFO )
    {
        cout << endl << "!!!!!!!!!!!!!!!!!!!! RESYNC !!!!!!!!!!!!!!!!!!!!" << endl;
    }

    // >>>>> Resync
    uint8_t dummyBuf[5];
    mSpiTR.rx_buf = (unsigned long)(dummyBuf); // First Packet has been read above
    mSpiTR.len = 5;
    mSpiTR.cs_change = 1; // Force deselect after "ioctl"

    ioctl( mSpiFd, SPI_IOC_MESSAGE(1), &mSpiTR );
    
    // Keeps /CS High for 185 msec according to datasheet
    std::this_thread::sleep_for(std::chrono::microseconds(185000));
    // <<<<< Resync
}

bool Lepton3::CciConnect()
{
    int result = LEP_OpenPort( mCciPort, LEP_CCI_TWI, 400, &mCciConnPort );

    if (result != LEP_OK)
    {
        cerr << "Cannot connect CCI port (I2C)" << endl;
        return false;
    }

    mCciConnected = true;
    return true;
}

float Lepton3::getSensorTemperatureK()
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return KELVIN;
    }

    LEP_SYS_FPA_TEMPERATURE_KELVIN_T temp;

    LEP_RESULT result = LEP_GetSysFpaTemperatureKelvin( &mCciConnPort, (LEP_SYS_FPA_TEMPERATURE_KELVIN_T_PTR)(&temp));

    if (result != LEP_OK)
    {
        cerr << "Cannot read lepton FPA temperature" << endl;
        return false;
    }

    float tempK = (float)(temp)/100.0f;

    if( mDebugLvl>=DBG_INFO )
        cout << "FPA temperature: " << tempK << "°K - " ;

    return tempK;
}


float Lepton3::raw2Celsius(float raw)
{
    float ambientTemperature = 25.0;
    float slope = 0.0217;

    return slope*raw+ambientTemperature-177.77;
}

bool Lepton3::lepton_perform_ffc()
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return false;
    }

    if( LEP_RunSysFFCNormalization(&mCciConnPort) != LEP_OK )
    {
        cerr << "Could not perform FFC Normalization" << endl;
        return false;
    }
}

int Lepton3::enableRadiometry( bool enable )
{
    if(!mCciConnected)
    {
        CciConnect();
    }

    LEP_RAD_ENABLE_E rad_status;

    if( LEP_GetRadEnableState(&mCciConnPort, (LEP_RAD_ENABLE_E_PTR)&rad_status ) != LEP_OK )
        return -1;

    LEP_RAD_ENABLE_E new_status = enable?LEP_RAD_ENABLE:LEP_RAD_DISABLE;

    if( rad_status != new_status )
    {
        if( LEP_SetRadEnableState(&mCciConnPort, new_status ) != LEP_OK )
            return -1;
    }

    return new_status;
}

void Lepton3::raw2data()
{
    int wordCount = mSpiRawFrameBufSize/2;
    int wordPackSize = mPacketSize/2;

    mMin = 0x3fff;
    mMax = 0;
    
    uint16_t* frameBuffer = (uint16_t*)mSpiRawFrameBuf;

    int pixIdx = 0;
    for(int i=0; i<wordCount; i++)
    {
        //skip the first 2 uint16_t's of every packet, they're 4 header bytes
        if(i % wordPackSize < 2)
        {
            continue;
        }

        //uint16_t value = (((uint16_t*)mSpiRawFrameBuf)[i]);
        
        int temp = mSpiRawFrameBuf[i*2];
			mSpiRawFrameBuf[i*2] = mSpiRawFrameBuf[i*2+1];
			mSpiRawFrameBuf[i*2+1] = temp;
			
	    uint16_t value = frameBuffer[i];
        
        //cout << value << " ";

        if( value > mMax )
        {
            mMax = value;
        }

        if(value < mMin)
        {
            if(value != 0)
                mMin = value;
        }

        mDataFrameBuf[pixIdx] = value;
        pixIdx++;
    }
    
    if( mDebugLvl>=DBG_INFO )
    {
        cout << endl << "Min: " << mMin << " - Max: " << mMax << endl;    
        cout << "---------------------------------------------------" << endl;
    }
    
    // cout << pixIdx << endl;
}

const uint16_t* Lepton3::getLastFrame( uint8_t& width, uint8_t& height, uint16_t* min/*=NULL*/, uint16_t* max/*=NULL*/ )
{
    if( !mDataValid )
        return NULL;

    mDataValid = false;
    
    width = FRAME_W;
    height = FRAME_H;

    if( min )
    {
        *min = mMin;
    }
    
    if( max )
    {
        *max = mMax;
    }
    
    return mDataFrameBuf;
}
