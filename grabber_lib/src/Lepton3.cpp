#include "Lepton3.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_RAD.h"
#include "LEPTON_OEM.h"

#include <bitset>

#define KELVIN (-273.15f)

#define FRAME_W 160
#define FRAME_H 120

using namespace std;

Lepton3::Lepton3(std::string spiDevice, uint16_t cciPort, DebugLvl dbgLvl )
    : mThread()
    , mSpiRawFrameBuf(NULL)
    , mDataFrameBuf16(NULL)
    , mDataFrameBufRGB(NULL)
    , mRgbEnabled(false)
{
    // >>>>> CCI
    mCciPort = cciPort;
    // <<<<< CCI

    // >>>>> VoSPI
    mSpiDevice = spiDevice;
    mSpiFd = -1;

    mSpiMode = SPI_MODE_3; // CPOL=1 (Clock Idle high level),
    // CPHA=1 (SDO transmit/change edge idle to active)
    mSpiBits = 8;
    mSpiSpeed = 32500000; // Max available SPI speed (according to Lepton3 datasheet)

    mSpiTR.tx_buf = (unsigned long)NULL;
    mSpiTR.delay_usecs = 50;
    mSpiTR.speed_hz = mSpiSpeed;
    mSpiTR.bits_per_word = mSpiBits;
    mSpiTR.cs_change = 0;
    mSpiTR.tx_nbits = 0;
    mSpiTR.rx_nbits = 0;
    mSpiTR.pad = 0;
    // <<<<< VoSPI

    setVoSPIData();

    // >>>>> Output Frame buffers
    mDataFrameBuf16 = new uint16_t[FRAME_W*FRAME_H];
    mDataFrameBufRGB = new uint8_t[FRAME_W*FRAME_H*3];
    // <<<<< Output Frame buffers

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

    if(mDataFrameBuf16)
        delete [] mDataFrameBuf16;

    if(mDataFrameBufRGB)
        delete [] mDataFrameBufRGB;
}

void Lepton3::setVoSPIData()
{
    mPacketCount  = 60;  // default no Telemetry

    // >>>>> Check Telemetry
    bool telemEnable;
    if( getTelemetryStatus( telemEnable ) == LEP_OK )
    {
        mPacketCount = 61;
    }
    // <<<<< Check Telemetry

    // >>>>> Check Packet Size
    LEP_OEM_VIDEO_OUTPUT_FORMAT_E format;
    if( getVideoOutputFormat( format ) == LEP_OK )
    {
        if( format==LEP_VIDEO_OUTPUT_FORMAT_RGB888 )
        {
            mPacketSize = 244;
            mRgbEnabled = true;
        }
        else
        {
            mPacketSize = 164;
            mRgbEnabled = false;
        }
    }
    else
    {
        mPacketSize   = 164; // default 14 bit raw data
        mRgbEnabled = false;
    }
    // <<<<< Check Packet Size


    mSegmentCount = 4;   // 4 segments for each unique frame

    mSegmSize = mPacketCount*mPacketSize;
    mSpiRawFrameBufSize = mSegmSize*mSegmentCount;

    if(mSpiRawFrameBuf)
    {
        delete [] mSpiRawFrameBuf;
        mSpiRawFrameBuf = NULL;
    }
    mSpiRawFrameBuf = new uint8_t[mSpiRawFrameBufSize];
    // <<<<< VoSPI data
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
        mBuffMutex.lock();

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

                    // >>>>> RAW to image data conversion
                    if( mDebugLvl>=DBG_FULL )
                    {
                        testTime2.tic();
                    }

                    if( mRgbEnabled )
                    {
                        raw2RGB();
                    }
                    else
                    {
                        raw2data16();
                    }

                    if( mDebugLvl>=DBG_FULL )
                    {
                        double elapsed = testTime2.toc();
                        cout << "VoSPI frame conversion time " << elapsed << " usec" << endl;
                    }

                    mDataValid = true;
                    // <<<<< RAW to image data conversion
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

        mBuffMutex.unlock();

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

LEP_RESULT Lepton3::getSensorTemperatureK(float& tempK )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_SYS_FPA_TEMPERATURE_KELVIN_T temp;

    LEP_RESULT result = LEP_GetSysFpaTemperatureKelvin( &mCciConnPort, (LEP_SYS_FPA_TEMPERATURE_KELVIN_T_PTR)(&temp));

    if (result != LEP_OK)
    {
        cerr << "Cannot read lepton FPA temperature" << endl;
        return LEP_ERROR;
    }

    tempK = static_cast<float>(temp)/100.0f;

    if( mDebugLvl>=DBG_INFO )
        cout << "FPA temperature: " << tempK << "°K - " ;

    return LEP_OK;
}


float Lepton3::raw2Celsius(float raw )
{
    float ambientTemperature = 25.0;
    float slope = 0.0217;

    return slope*raw+ambientTemperature-177.77;
}

LEP_RESULT Lepton3::lepton_perform_ffc()
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    if( LEP_RunSysFFCNormalization(&mCciConnPort) != LEP_OK )
    {
        cerr << "Could not perform FFC Normalization" << endl;
        return LEP_ERROR;
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::enableTelemetry( bool enable )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    mBuffMutex.lock();

    LEP_SYS_TELEMETRY_ENABLE_STATE_E tel_status;

    if( LEP_GetSysTelemetryEnableState(&mCciConnPort, (LEP_SYS_TELEMETRY_ENABLE_STATE_E_PTR)&tel_status ) != LEP_OK )
    {
        cerr << "Cannot read Telemetry status" << endl;

        mBuffMutex.unlock();
        return LEP_ERROR;
    }

    LEP_SYS_TELEMETRY_ENABLE_STATE_E new_status = enable?LEP_TELEMETRY_ENABLED:LEP_TELEMETRY_DISABLED;

    if( tel_status != new_status )
    {
        if( LEP_SetSysTelemetryEnableState(&mCciConnPort, new_status ) != LEP_OK )
        {
            cerr << "Cannot set Telemetry status" << endl;

            mBuffMutex.unlock();
            return LEP_ERROR;
        }

        setVoSPIData();
    }

    mBuffMutex.unlock();
    return LEP_OK;
}

LEP_RESULT Lepton3::getTelemetryStatus( bool &status )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_SYS_TELEMETRY_ENABLE_STATE_E tel_status;

    if( LEP_GetSysTelemetryEnableState(&mCciConnPort, (LEP_SYS_TELEMETRY_ENABLE_STATE_E_PTR)&tel_status ) != LEP_OK )
    {
        cerr << "Cannot read Telemetry status" << endl;
        return LEP_ERROR;
    }

    if( tel_status == LEP_TELEMETRY_ENABLED )
        status = true;
    else
        status = false;

    return LEP_OK;
}


LEP_RESULT Lepton3::getRadiometryStatus( bool& status )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_RAD_ENABLE_E rad_status;

    if( LEP_GetRadEnableState(&mCciConnPort, (LEP_RAD_ENABLE_E_PTR)&rad_status ) != LEP_OK )
    {
        cerr << "Cannot read Radiometry status" << endl;
        return LEP_ERROR;
    }

    if( rad_status == LEP_RAD_ENABLE )
        status = true;
    else
        status = false;

    return LEP_OK;
}

LEP_RESULT Lepton3::enableRadiometry( bool enable )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_RAD_ENABLE_E rad_status;

    if( LEP_GetRadEnableState(&mCciConnPort, (LEP_RAD_ENABLE_E_PTR)&rad_status ) != LEP_OK )
    {
        cerr << "Cannot read Radiometry status" << endl;
        return LEP_ERROR;
    }

    LEP_RAD_ENABLE_E new_status = enable?LEP_RAD_ENABLE:LEP_RAD_DISABLE;

    if( rad_status != new_status )
    {
        if( LEP_SetRadEnableState(&mCciConnPort, new_status ) != LEP_OK )
        {
            cerr << "Cannot set Radiometry status" << endl;
            return LEP_ERROR;
        }
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::getAgcStatus(bool &status)
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_AGC_ENABLE_E agc_status;

    if( LEP_GetAgcEnableState( &mCciConnPort, (LEP_AGC_ENABLE_E_PTR)&agc_status ) != LEP_OK )
    {
        cerr << "Cannot read AGC status" << endl;
        return LEP_ERROR;
    }

    if( agc_status == LEP_AGC_ENABLE )
        status = true;
    else
        status = false;

    return LEP_OK;
}

LEP_RESULT Lepton3::enableAgc( bool enable )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_AGC_ENABLE_E agc_status;

    if( LEP_GetAgcEnableState(&mCciConnPort, (LEP_AGC_ENABLE_E_PTR)&agc_status ) != LEP_OK )
    {
        cerr << "Cannot read AGC status" << endl;
        return LEP_ERROR;
    }

    LEP_AGC_ENABLE_E new_status = enable?LEP_AGC_ENABLE:LEP_AGC_DISABLE;

    if( agc_status != new_status )
    {
        if( LEP_SetAgcEnableState(&mCciConnPort, new_status ) != LEP_OK )
        {
            cerr << "Cannot set Radiometry status" << endl;
            return LEP_ERROR;
        }
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::getGainMode( LEP_SYS_GAIN_MODE_E& mode)
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    if( LEP_GetSysGainMode( &mCciConnPort, (LEP_SYS_GAIN_MODE_E_PTR)&mode ) != LEP_OK )
    {
        cerr << "Cannot read Sys Gain Mode" << endl;
        return LEP_ERROR;
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::getVideoOutputFormat( LEP_OEM_VIDEO_OUTPUT_FORMAT_E& format  )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    if( LEP_GetOemVideoOutputFormat( &mCciConnPort, (LEP_OEM_VIDEO_OUTPUT_FORMAT_E_PTR)&format ) != LEP_OK )
    {
        cerr << "Cannot read Video Output format" << endl;
        return LEP_ERROR;
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::enableRgbOutput( bool enable )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    mBuffMutex.lock();

    LEP_OEM_VIDEO_OUTPUT_FORMAT_E format,newFormat;

    if( getVideoOutputFormat( format) != LEP_OK )
    {
        mBuffMutex.unlock();
        return LEP_ERROR;
    }

    if( enable )
    {
        newFormat = LEP_VIDEO_OUTPUT_FORMAT_RGB888;
    }
    else
    {
        newFormat = LEP_VIDEO_OUTPUT_FORMAT_RAW14;
    }

    if( enableAgc( enable ) != LEP_OK ) // RGB888 requires AGC enabled
    {
        mBuffMutex.unlock();
        return LEP_ERROR;
    }

    if( newFormat != format )
    {
        if( LEP_SetOemVideoOutputFormat( &mCciConnPort, newFormat ) != LEP_OK )
        {
            cerr << "Cannot set Video Output format" << endl;

            mBuffMutex.unlock();
            return LEP_ERROR;
        }

        // Update data buffers!
        setVoSPIData();
    }
    mBuffMutex.unlock();
}

/*LEP_RESULT Lepton3::getSpotROI( uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_RAD_ROI_T roi;

    if( LEP_GetRadSpotmeterRoi( &mCciConnPort, (LEP_RAD_ROI_T_PTR)&roi ) != LEP_OK )
    {
        cerr << "Cannot read Spotmeter ROI" << endl;
        return LEP_ERROR;
    }

    x = roi.startCol;
    y = roi.startRow;
    w = roi.endCol - x;
    h = roi.endRow - y;

    return LEP_OK;
}

LEP_RESULT Lepton3::setSpotROI( uint16_t x, uint16_t y, uint16_t w, uint16_t h )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_RAD_ROI_T newROI;
    newROI.startCol = x;
    newROI.startRow = y;
    newROI.endCol = x+w;
    newROI.endRow = y+h;

    if( LEP_SetRadSpotmeterRoi( &mCciConnPort, newROI ) != LEP_OK )
    {
        cerr << "Cannot set Spotmeter ROI" << endl;
        return LEP_ERROR;
    }

    return LEP_OK;
}

LEP_RESULT Lepton3::getSpotInfo( float& valueK, float& minK, float& maxK, uint16_t& count )
{
    if(!mCciConnected)
    {
        if( !CciConnect() )
            return LEP_ERROR;
    }

    LEP_RAD_SPOTMETER_OBJ_KELVIN_T info;

    if( LEP_GetRadSpotmeterObjInKelvinX100( &mCciConnPort, (LEP_RAD_SPOTMETER_OBJ_KELVIN_T_PTR)&info ) != LEP_OK )
    {
        cerr << "Cannot read Spotmeter info" << endl;
        return LEP_ERROR;
    }

    valueK = static_cast<float>(info.radSpotmeterValue)/100.f;
    minK = static_cast<float>(info.radSpotmeterMinValue)/100.f;
    maxK = static_cast<float>(info.radSpotmeterMaxValue)/100.f;
    count = info.radSpotmeterPopulation;

    return LEP_OK;
}*/

void Lepton3::raw2data16()
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

        mDataFrameBuf16[pixIdx] = value;
        pixIdx++;
    }
    
    if( mDebugLvl>=DBG_INFO )
    {
        cout << endl << "Min: " << mMin << " - Max: " << mMax << endl;
        cout << "---------------------------------------------------" << endl;
    }
    
    // cout << pixIdx << endl;
}

void Lepton3::raw2RGB()
{
    int byteCount = mSpiRawFrameBufSize;
    int pxPackSize = mPacketSize;

    uint8_t* frameBuffer = mSpiRawFrameBuf;

    int pixIdx = 0;
    for(int i=0; i<byteCount; i++)
    {
        //skip the first 4 uint8_t's of every packet, they're 4 header bytes
        if(i % pxPackSize < 4)
        {
            continue;
        }

        mDataFrameBufRGB[pixIdx] = mSpiRawFrameBuf[i];
        pixIdx++;
    }

    // cout << pixIdx << endl;
}

const uint16_t* Lepton3::getLastFrame16( uint8_t& width, uint8_t& height, uint16_t* min/*=NULL*/, uint16_t* max/*=NULL*/ )
{
    if( mRgbEnabled )
        return NULL;

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
    
    return mDataFrameBuf16;
}

const uint8_t* Lepton3::getLastFrameRGB( uint8_t& width, uint8_t& height )
{
    if( !mRgbEnabled )
        return NULL;

    if( !mDataValid )
        return NULL;

    mDataValid = false;

    width = FRAME_W;
    height = FRAME_H;

    return mDataFrameBufRGB;
}
