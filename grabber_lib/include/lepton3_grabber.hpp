#include <string>

#include <thread>
#include <mutex>
#include <chrono>

#define FRAME_W 160
#define FRAME_H 120

#define VOSPI_FRAME_SIZE (164)
#define LEP_SPI_BUFFER (118080) //(118080)39360
/* modify /boot/cmdline.txt to include spidev.bufsiz=131072 */

class L3_grabber
{
public:
    L3_grabber(std::string SpiDevice="/dev/spidev1.0");
    ~L3_grabber();

    bool start();

    const char* getLastFrame(int* outW=NULL, int* outH=NULL);

protected:
    void acquire_thread_func();

    int SpiTransfer(int fd);
    void frameConvert();
    
private:
    std::timed_mutex mFrameMutex;
    std::thread mThread;

    int mFrameTimeoutMsec;
    
    const uint8_t mFrameW;
    const uint8_t mFrameH;
    
    char mLastFrame[FRAME_W*FRAME_H];
    
    bool mStop;

    // >>>>> VideoSPI
    std::string mSpiDevice;

    uint8_t mSpiMode = SPI_CPOL | SPI_CPHA;
    uint8_t mSpiBits = 8;
    uint32_t mSpiSpeed = 20000000;
    uint16_t mSpiDelay = 65535;
    uint8_t mSpiStatusBits = 0;

    int8_t mSpiLastPacket = -1;


    uint8_t mSpiRxBuf[LEP_SPI_BUFFER] = {0};
    uint16_t mSpiLeptonImg[240][80];
    // <<<<< VideoSPI
};

