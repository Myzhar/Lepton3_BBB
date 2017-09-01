#ifndef LEPTON3
#define LEPTON3

#include <ctime>
#include <stdint.h>

#include <iostream>
#include <cstdlib>
#include <thread>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "LEPTON_Types.h"

#include "stopwatch.hpp"

class Lepton3
{
public:
    typedef enum _debug_lvl { DBG_NONE=0, DBG_INFO=1, DBG_FULL=2 } DebugLvl;

    Lepton3(std::string spiDevice="/dev/spidev1.0", uint16_t cciPort=1, DebugLvl dbgLvl=DBG_FULL);
    ~Lepton3();

    bool start();
    void stop();

protected:
    void thread_func();

    bool SpiOpenPort(); //!< Opens VoSPI port
    void SpiClosePort(); //!< Closes VoSPI port
    int SpiReadSegment(); //!< Reads a VoSPI segment and returns its ID

public:

private:
    // >>>>> VoSPI
    std::string mSpiDevice; //!< SPI port device name
    int mSpiFd; //!< SPI descriptor
    unsigned char mSpiMode; //!< SPI mode
    unsigned char mSpiBits; //!< SPI bits per words
    unsigned int mSpiSpeed; //!< SPI max speed

    uint8_t mPacketCount; //!< VoSPI Packet for each segment
    uint8_t mPacketSize; //!< VoSPI Packet size in bytes
    uint8_t mSegmentCount; //!< VoSPI segment for each frame

    uint8_t* mSpiResultBuf; //!< VoSPI read buffer for single segment
    uint16_t* frameBuffer;
    uint8_t* grayBuffer;

    double mSegmentFreq; //!< Segment output frequency
    // <<<<< VoSPI

    // >>>>> Lepton control (CCI)
    bool mCciConnected;
    LEP_UINT16 mCciPort;
    LEP_CAMERA_PORT_DESC_T_PTR mCciConnPort;

    bool CciConnect();
    bool lepton_perform_ffc();
    float getSensorTemperatureK();
    float raw2Celsius(float);
    int enableRadiometry( bool enable );
    // <<<<< Lepton control (CCI)

    std::thread mThread;

    bool mStop;

    DebugLvl mDebugLvl;
    
    StopWatch mThreadWatch;
};



#endif // LEPTONTHREAD
