#include "LeptonThread.hpp"
#include "Palettes.h"
#include "SPI.h"
#include "Lepton_I2C.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

//static const char *device = "/dev/spidev0.0";
//uint8_t mode;
//static uint8_t bits = 8;
//static uint32_t speed = 20000000;
int snapshotCount = 0;
int frame = 0;
static int raw [120][160];

using namespace std;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

LeptonThread::LeptonThread( std::string spiDevice ) 
	: mThread()
{
	mSpiDevice = spiDevice;	
	
	mStop = false;
}

LeptonThread::~LeptonThread() 
{
	stop();
}

bool LeptonThread::start()
{
    mThread = std::thread( &LeptonThread::thread_func, this );
}

void LeptonThread::stop()
{
	mStop = true;

    if(mThread.joinable())
    {
        mThread.join();
    }
}

void LeptonThread::thread_func()
{
	cout << "Grabber thread started ..." << endl;
	
	mStop = false;
	
	/*/create the initial image
	QRgb red = qRgb(255,0,0);
	myImage = QImage(160, 120, QImage::Format_RGB888);
	for(int i=0;i<160;i++) {
		for(int j=0;j<120;j++) {
			myImage.setPixel(i, j, red);
		}
	}*/
	
	int ret = 0;
	int fd = 0;
	
	fd = SpiOpenPort(mSpiDevice);
	
	cout << "SPI fd: " << fd << endl;

	/*fd = open(mSpiDevice.c_str(), O_RDWR);
	if (fd < 0)
	{
		pabort("can't open device");
	}

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't get spi mode");
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't get bits per word");
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't get max speed hz");
	}*/

	//open spi port
	//emit updateImage(myImage);
	
	// Enable Radiometry
	if( enable_radiometry( true ) < 0 )
	{
		cout << "Failed to enable radiometry" << endl;
	}
	else
	{
		cout << "Radiometry enabled" << endl;
	}

	while(true) 
	{
		int resets = 0;
		int segmentNumber = 0;
		
		cout << "0" << endl;
		
		for(int i = 0; i < NUMBER_OF_SEGMENTS; i++)
		{
			for(int j=0;j<PACKETS_PER_SEGMENT;j++) 
			{
				if( mStop )
        		{
            		break;
        		}
        		
				//read data packets from lepton over SPI
				read(fd, result+sizeof(uint8_t)*PACKET_SIZE*(i*PACKETS_PER_SEGMENT+j), sizeof(uint8_t)*PACKET_SIZE);
				int packetNumber = result[((i*PACKETS_PER_SEGMENT+j)*PACKET_SIZE)+1];		
				
				//cout << "Packet: " << packetNumber << endl;	 
				
				//if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
				if(packetNumber != j) 
				{
					//cout << "*** Drop *** Packet: " << packetNumber << " - j:" << j << endl;
					
					j = -1;
					resets += 1;
					//usleep(1000);
					continue;
					
					if(resets == 1000) 
					{
						SpiClosePort(fd);
						cout << "restarting spi..." << endl;
						usleep(5000);
						SpiOpenPort(mSpiDevice);
						
						resets = 0;
					}
				} 
				else if(packetNumber == 20) 
				{
					//reads the "ttt" number
					segmentNumber = result[(i*PACKETS_PER_SEGMENT+j)*PACKET_SIZE] >> 4;
					
					//cout << "SegmentNumber: " << segmentNumber << endl;
					
					//if it's not the segment expected reads again
					if(segmentNumber == 0)
					{
						//cout << "*** Drop *** SegmentNumber: " << segmentNumber << endl;
						
						j = -1;
						//resets += 1;
						//usleep(1000);
					}
				}			
			}		
			usleep(10);
			
			if( mStop )
    		{
        		break;
    		}
		}
	/*	if(resets >= 30) {
			cout << "done reading, resets: " << resets << endl;
		}*/
		
		if( mStop )
        {
            cout << "... grabber thread stopped ..." << endl;
            break;
        }
		
		cout << "1" << endl;

		frameBuffer = (uint16_t *)result;
		int row, column;
		uint16_t value;
		uint16_t minValue = 65535;
		uint16_t maxValue = 0;

		
		for(int i=0;i<FRAME_SIZE_UINT16;i++) 
		{
			//skip the first 2 uint16_t's of every packet, they're 4 header bytes
			if(i % PACKET_SIZE_UINT16 < 2) 
			{
				continue;
			}
			
			//flip the MSB and LSB at the last second
			int temp = result[i*2];
			result[i*2] = result[i*2+1];
			result[i*2+1] = temp;
			
			value = frameBuffer[i];
			if(value> maxValue) 
			{
				maxValue = value;
			}
			if(value < minValue) 
			{
				if(value != 0)
					minValue = value;
			}		
		}
		
		cout << "2" << endl;
		
		std::cout << "T Min: " << raw2Celsius(minValue) <<" °C"<<std::endl;	
		std::cout << "T Max: " << raw2Celsius(maxValue) <<" °C"<<std::endl;	
		float diff = maxValue - minValue;
		float scale = 255/diff;
		//QRgb color;
		float valueCenter = 0;
		
		for(int k=0; k<FRAME_SIZE_UINT16; k++) 
		{
			if(k % PACKET_SIZE_UINT16 < 2) 
			{
				continue;
			}
		
			value = (frameBuffer[k] - minValue) * scale;
			//printf("%u\n", frameBuffer[k]);
			
			const int *colormap = colormap_glowBow;
			//color = qRgb(colormap[3*value], colormap[3*value+1], colormap[3*value+2]);
			
				if((k/PACKET_SIZE_UINT16) % 2 == 0)
				{//1
					column = (k % PACKET_SIZE_UINT16 - 2);
					row = (k / PACKET_SIZE_UINT16)/2;
				}
				else
				{//2
					column = ((k % PACKET_SIZE_UINT16 - 2))+(PACKET_SIZE_UINT16-2);
					row = (k / PACKET_SIZE_UINT16)/2;
				}	
				raw[row][column] = frameBuffer[k];
				if(column == 160 && row == 120)
					valueCenter = frameBuffer[k];
								
				//myImage.setPixel(column, row, color);
				
		}
		//drawSquare(valueCenter);
		//lets emit the signal for update
		//emit updateImage(myImage);
		frame++;
		/*if(frame == 5)
		{
			snapshot();
			//abort();
		}*/
		
		cout << "3" << endl;
	}
	
	//finally, close SPI port just bcuz
	SpiClosePort(fd);
	
	cout << "... grabber thread finished" << endl;
}

/*void LeptonThread::drawSquare(float ){
	QPainter paint(&myImage);
	paint.setPen(Qt::blue);
	//paint.drawRect(79,59,2,2);
	//paint.setPen(Qt::black);
	paint.setFont(QFont("Arial", 8));
	//paint.drawText(84,64, QString("%1 C").arg(raw2Celsius(value)));
}*/

/*void LeptonThread::snapshot(){
	snapshotCount++;
	//----------------------- criando imagem ------------------------
	//variables
	struct stat buf;
	const char *inicio = "rgb";
	const char *fim = ".png";
	char meio[32];
	//convert from int to string
	sprintf(meio, "%d", snapshotCount);
	char name[64];
	//appending photo name
	strcpy(name, inicio);
	strcat(name, meio);
	strcat(name, fim);
	//if this name already exists
	int exists = stat(name,&buf);
	//if the name exists stat returns 0
		while(exists == 0){
			//try next number
			snapshotCount++;
			strcpy(name, inicio);
			sprintf(meio, "%d", snapshotCount);
			strcat(name, meio);
			strcat(name, fim);
			exists = stat(name, &buf);
		}
	//saving photo
	myImage.save(QString(name), "PNG", 100);
	
	//---------------------- criando txt -----------------------
	//creating file name
	fim = ".txt";
	strcpy(name, inicio);
	strcat(name, meio);
	strcat(name, fim);
	
	FILE *arq = fopen(name,"wt");
	char valor[64];

	for(int i = 0; i < 120; i++)
	{
			for(int j = 0; j < 160; j++)
			{
				sprintf(valor, "%f", raw2Celsius(raw[i][j]));
				fputs(valor, arq);
				fputs(" ", arq);
			}
			fputs("\n", arq);
	}
	fclose(arq);
}*/


void LeptonThread::performFFC() {
	//perform FFC
	lepton_perform_ffc();
}
