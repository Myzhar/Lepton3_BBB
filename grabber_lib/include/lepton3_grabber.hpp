#include <string>

#include <thread>
#include <mutex>
#include <chrono>

#define FRAME_W 160
#define FRAME_H 120

class l3_grabber
{
public:
    l3_grabber(std::string spi_port="/dev/spidev1.0");
    ~l3_grabber();

    bool start();

    const char* getLastFrame(int* outW=NULL, int* outH=NULL);

protected:
    void acquire_func();
    
private:
    std::timed_mutex mFrameMutex;
    std::thread mThread;

    int mFrameTimeoutMsec;
    
    const uint8_t mFrameW;
    const uint8_t mFrameH;
    
    char mLastFrame[FRAME_W*FRAME_H];
    
    bool mStop;
};

