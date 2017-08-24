#ifndef LEPTONTHREAD
#define LEPTONTHREAD

#include <ctime>
#include <stdint.h>

//#include <QThread>
//#include <QtCore>
//#include <QPixmap>
//#include <QImage>
//#include <QString>
#include <iostream>
//#include <QPainter>
#include <cstdlib>
#include <thread>


#define PACKET_SIZE 164  //Bytes
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)   
#define NUMBER_OF_SEGMENTS 4
#define PACKETS_PER_SEGMENT 60
#define PACKETS_PER_FRAME (PACKETS_PER_SEGMENT*NUMBER_OF_SEGMENTS)
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
//#define FPS 27;

class LeptonThread
{
  //Q_OBJECT;

public:
  LeptonThread(std::string spiDevice="/dev/spidev1.0");
  ~LeptonThread();
  
  bool start();
  void stop();

protected:
  void thread_func();

public/* slots*/:
  void performFFC();
  //void snapshot();
  //void drawSquare(float);

//signals:
//  void updateText(QString);
//  void updateImage(QImage);

private:
  std::string mSpiDevice;
  //QImage myImage;

  uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
  uint16_t* frameBuffer;
  uint8_t* grayBuffer;

  std::thread mThread;
  
  bool mStop;
};

#endif // LEPTONTHREAD
