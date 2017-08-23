#include "lepton3_grabber.hpp"
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <string.h>
#include <time.h>

using namespace std;
using msec = std::chrono::milliseconds;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

L3_grabber::L3_grabber( std::string SpiDevice )
    : mThread()
    , mFrameW(FRAME_W)
    , mFrameH(FRAME_H)
{
    mStop = false;

    mSpiDevice = SpiDevice;

    mSpiMode = SPI_CPOL | SPI_CPHA;
    mSpiBits = 8;
    mSpiSpeed = 20000000;
    mSpiDelay = 65535;
    mSpiStatusBits = 0;
}

L3_grabber::~L3_grabber()
{
    mStop = true;

    if(mThread.joinable())
    {
        mThread.join();
    }
}

bool L3_grabber::start()
{
    mThread = std::thread( &L3_grabber::acquire_thread_func, this );
}

const char* L3_grabber::getLastFrame(int* outW, int* outH)
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

void L3_grabber::acquire_thread_func()
{
    cout << "Acquire thread started ..." << endl;

    // >>>>> Open SPI
    int ret = 0;
    int fd;

    fd = open(mSpiDevice.c_str(), O_RDWR);
    if (fd < 0)
    {
        cout << "Can't open device: " << mSpiDevice << endl;
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mSpiMode);
    if (ret == -1)
    {
        cout << "can't set spi mode" << endl;
    }

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mSpiMode);
    if (ret == -1)
    {
        cout << "can't get spi mode" << endl;
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &mSpiBits);
    if (ret == -1)
    {
        cout << "can't set bits per word" << endl;
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &mSpiBits);
    if (ret == -1)
    {
        cout << "can't get bits per word" << endl;
    }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &mSpiSpeed);
    if (ret == -1)
    {
        cout << "can't set max speed hz" << endl;
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &mSpiSpeed);
    if (ret == -1)
    {
        cout << "can't get max speed hz" << endl;
    }

    cout << "spi mode: " << mSpiMode << endl;
    cout << "bits per word: " << mSpiBits  << endl;
    cout << "max speed: " << mSpiSpeed << " Hz ("<< mSpiSpeed/1000 << " KHz)" << endl;
    // <<<<< Open SPI

    while(1)
    {
        if( mStop )
        {
            cout << "... Acquire thread stopped ..." << endl;
            break;
        }

        // >>>>> Get frame from VoSPI
        while(mSpiStatusBits != 0x0f)
        {
            SpiTransfer(fd);
        }
        // <<<<< Get frame from VoSPI


        // >>>>> Convert frame to image
        mFrameMutex.lock();
        frameConvert();
        mFrameMutex.unlock();
        // <<<<< Convert frame to image

    }

    close(fd);

    cout << "... acquire thread finished" << endl;
}

/**************************************************************************************
* Original code modified from SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
****************************************
Modified for Lepton by:
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************
July 2017
Modified by Luke Van Horn for Lepton 3

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License.
 **************************************************************************************/



int L3_grabber::SpiTransfer(int fd)
{
    int ret;
    int i;
    int ip;
    uint8_t packet_number = 0;
    uint8_t segment = 0;
    uint8_t current_segment = 0;
    int packet = 0;
    int state = 0;  //set to 1 when a valid segment is found
    int pixel = 0;

    spi_ioc_transfer tr;

    tr.tx_buf = (unsigned long)NULL;
    tr.rx_buf = (unsigned long)mSpiRxBuf;
    tr.len = LEP_SPI_BUFFER;
    tr.delay_usecs = mSpiDelay;
    tr.speed_hz = mSpiSpeed;
    tr.bits_per_word = mSpiBits;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        cout << "Can't read spi data" << endl;

        return -1;
    }

    for(ip = 0; ip < (ret / VOSPI_FRAME_SIZE); ip++)
    {
        packet = ip * VOSPI_FRAME_SIZE;

        //check for invalid packet number
        if((mSpiRxBuf[packet] & 0x0f) == 0x0f)
        {
            state = 0;
            continue;
        }

        packet_number = mSpiRxBuf[packet + 1];

        if(packet_number > 0 && state == 0)
        {
            continue;
        }

        if(state == 1 && packet_number == 0)
        {
            state = 0;  //reset for new segment
        }

        //look for the start of a segment
        if(state == 0 && packet_number == 0 && (packet + (20 * VOSPI_FRAME_SIZE)) < ret)
        {
            segment = (mSpiRxBuf[packet + (20 * VOSPI_FRAME_SIZE)] & 0x70) >> 4;
            if(segment > 0 && segment < 5 && mSpiRxBuf[packet + (20 * VOSPI_FRAME_SIZE) + 1] == 20)
            {
                state = 1;
                current_segment = segment;
                cout << "new segment: " << segment << endl;
            }
        }

        if(!state)
        {
            continue;
        }

        for(i = 4; i < VOSPI_FRAME_SIZE; i+=2)
        {
            pixel = packet_number + ((current_segment - 1) * 60);
            mSpiLeptonImg[pixel][(i - 4) / 2] = (mSpiRxBuf[packet + i] << 8 | mSpiRxBuf[packet + (i + 1)]);
        }

        if(packet_number == 59)
        {
            //set the segment status bit
            mSpiStatusBits |= ( 0x01 << (current_segment - 1));
        }
    }

    return mSpiStatusBits;
}

void L3_grabber::frameConvert()
{
    uint16_t maxval = 0;
    uint16_t minval = USHRT_MAX;

    printf("Calculating min/max values for proper scaling...\n");

    for(int i = 0; i < 240; i++)
    {

        for(int j = 0; j < 80; j++)
        {
            uint16_t value = mSpiLeptonImg[i][j];

            if( value > maxval )
            {
                maxval = value;
            }
            if( value < minval )
            {
                minval = value;
            }
        }
    }

    cout << "maxval = " << maxval << endl;
    cout << "minval = " << minval << endl;

    //fprintf(f,"P2\n160 120\n%u\n",maxval-minval);


    for(int i=0; i < 240; i += 2)
    {
        int idx = 0;

        /* first 80 pixels in row */
        for(int j = 0; j < 80; j++)
        {
            mLastFrame[idx] = mSpiLeptonImg[i][j] - minval;
            ++idx;
            //fprintf(f,"%d ", lepton_image[i][j] - minval);
        }

        idx = 0;

        /* second 80 pixels in row */
        for(int j = 0; j < 80; j++)
        {
            mLastFrame[idx] = mSpiLeptonImg[i + 1][j] - minval;
            ++idx;
            //fprintf(f,"%d ", lepton_image[i + 1][j] - minval);
        }
        //fprintf(f,"\n");
    }
    //fprintf(f,"\n\n");
}
