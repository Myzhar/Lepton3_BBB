#include "SPI.h"
#include <iostream>

unsigned char spi_mode = SPI_MODE_3;
unsigned char spi_bitsPerWord = 8;
unsigned int spi_speed = 20000000;

using namespace std;

int SpiOpenPort (std::string spi_device, int debug_out)
{
	int status_value = -1;
	int spi_cs_fd;


	//----- SET SPI MODE -----
	//SPI_MODE_0 (0,0)  CPOL=0 (Clock Idle low level), CPHA=0 (SDO transmit/change edge active to idle)
	//SPI_MODE_1 (0,1)  CPOL=0 (Clock Idle low level), CPHA=1 (SDO transmit/change edge idle to active)
	//SPI_MODE_2 (1,0)  CPOL=1 (Clock Idle high level), CPHA=0 (SDO transmit/change edge active to idle)
	//SPI_MODE_3 (1,1)  CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
	spi_mode = SPI_MODE_3;

	//----- SET BITS PER WORD -----
	spi_bitsPerWord = 8;

	//----- SET SPI BUS SPEED -----
		
	if( debug_out!=0 )
		cout << "Opening SPI device: " << spi_device << endl;
	spi_cs_fd = open(spi_device.c_str(), O_RDWR);

	if (spi_cs_fd < 0)
	{
		perror("Error - Could not open SPI device");
		return -1;
	}

	status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
	if(status_value < 0)
	{
		perror("Could not set SPIMode (WR)...ioctl fail");
		return -1;
	}

	status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
	if(status_value < 0)
	{
		perror("Could not set SPIMode (RD)...ioctl fail");
		return -1;
	}
	
	if( debug_out!=0 )
		cout << "SPI mode: " << (int)spi_mode << endl;

	status_value = ioctl(spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
	if(status_value < 0)
	{
		perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
		return -1;
	}

	status_value = ioctl(spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
	if(status_value < 0)
	{
		perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
		return -1;
	}
	
	if( debug_out!=0 )
		cout << "SPI bits per word: " << (int)spi_bitsPerWord << endl;

	status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if(status_value < 0)
	{
		perror("Could not set SPI speed (WR)...ioctl fail");
		return -1;
	}

	status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
	if(status_value < 0)
	{
		perror("Could not set SPI speed (RD)...ioctl fail");
		return -1;
	}
	
	if( debug_out!=0 )
		cout << "SPI max speed: " << (int)spi_speed << endl;
	
	return(spi_cs_fd);
}

int SpiClosePort( int spi_fd )
{
	int status_value = -1;


	status_value = close(spi_fd);
	if(status_value < 0)
	{
		perror("Error - Could not close SPI device");
		return -1;
	}
	
	return(status_value);
}
