#include "Lepton3.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "Lepton_I2C.h"
#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_RAD.h"

#define KELVIN (-273.15f)

using namespace std;

Lepton3::Lepton3(std::string spiDevice, uint16_t cciPort, DebugLvl dbgLvl )
	: mThread()
    , mSpiResultBuf(NULL)
{
    // >>>>> VoSPI
	mSpiDevice = spiDevice;	
    mSpiFd = -1;

    mSpiMode = SPI_MODE_3; // CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
    mSpiBits = 8;
    mSpiSpeed = 20000000; // Max available SPI speed (according to Lepton3 datasheet)

    mFragmentCount=60; // default no Telemetry
    mFragmentSize=164; // default 14 bit raw data
    mSegmentCount=4; // 4 segments for each unique frame

    mSegmentFreq=106.0f; // According to datasheet each segment is ready at 106 Hz

    mSpiResultBuf = new uint8_t[mFragmentCount*mFragmentSize];
    // <<<<< VoSPI

    // >>>>> CCI
    mCciConnected = false;
    mCciPort = cciPort;
    // <<<<< CCI

    mDebugLvl = dbgLvl;

    if( mDebugLvl>=DBG_INFO )
        cout << "Debug level: " << mDebugLvl << endl;

	mStop = false;
}

Lepton3::~Lepton3()
{
	stop();

    if(mSpiResultBuf)
        delete [] mSpiResultBuf;
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
        cerr << "Error - Could not open SPI device: ") << mSpiDevice.c_str() << endl;
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

    // >>>>> Segment reading
    int segmentSize = mFragmentCount*mFragmentSize;

    if( read( mSpiFd, mSpiResultBuf, segmentSize ) != segmentSize )
    {
        cerr << "Error reading full segment from SPI" << endl;
        return -1;
    }
    // <<<<< Segment reading

    // >>>>> Segment ID
    // Segment ID is written in the 20th fragment int the bit 1-3 of the first byte (the first bit is always 0)
    // Packet number is written in the bit 4-7 of the first byte

    int pktNumber = (mSpiResultBuf[20*mFragmentSize] & 0x0F);
    if( pktNumber!=20 )
    {
        if( mDebugLvl>=DBG_INFO )
        {
            cout << "Wrong 20th fragment in segment" << endl;
            return -1
        }
    }

    int segmentID = (mSpiResultBuf[20*mFragmentSize] & 0x70) >> 4;
    // <<<<< Segment ID

    if( mDebugLvl>=DBG_FULL )
    {
        cout << "Segment ID:" << segmentID << endl;
    }

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

	while(true) 
	{		
        int segment = SpiReadSegment();
	}
	
	//finally, close SPI port just bcuz
    SpiClosePort();
	
    if( mDebugLvl>=DBG_INFO )
        cout << "... grabber thread finished" << endl;
}

bool Lepton3::CciConnect()
{
    int result = LEP_OpenPort( mCciPort, LEP_CCI_TWI, 400, mCciConnPort );

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

    LEP_RESULT result = LEP_GetSysFpaTemperatureKelvin( mCciConnPort, (LEP_SYS_FPA_TEMPERATURE_KELVIN_T_PTR)(&temp));

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

    if( lp_o LEP_RunSysFFCNormalization(mCciConnPort);
}

int Lepton3::enableRadiometry( bool enable )
{
    if(!mCciConnected)
    {
        CciConnect();
    }

    LEP_RAD_ENABLE_E rad_status;

    if( LEP_GetRadEnableState(mCciConnPort, (LEP_RAD_ENABLE_E_PTR)&rad_status ) != LEP_OK )
        return -1;

    LEP_RAD_ENABLE_E new_status = enable?LEP_RAD_ENABLE:LEP_RAD_DISABLE;

    if( rad_status != new_status )
    {
        if( LEP_SetRadEnableState(mCciConnPort, new_status ) != LEP_OK )
            return -1;
    }

    return new_status;
}
