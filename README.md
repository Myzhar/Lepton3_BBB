# Lepton3_BBB
A grabber for BeagleBone Blue to acquire thermal images from Lepton3 sensor

Demo: [Video 1](https://youtu.be/Yov98Ps2ttc) - [Video 2](https://youtu.be/WnkK0AOtyL8) - [Video 2](https://youtu.be/WnkK0AOtyL8)

## Change SPI Buffer size of the BeagleBone board
SPI buffer size must be set to ~~131072~~ 20480 bytes (safe value) to allow the receiving of a full segment for RGB data

(61 packets x 240 bytes = 14640 bytes)

```
$ sudo nano /boot/uEnv.txt
```
Search for the following line:
```
cmdline=coherent_pool=1M net.ifnames=0 quiet cape_universal=enable
```
add ``` spidev.bufsiz=20480 ```
at the end of the line, e.g.:
```
cmdline=coherent_pool=1M net.ifnames=0 quiet cape_universal=enable spidev.bufsiz=20480
```
Save, exit and reboot

## Compile the code
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```
## Enable full performances of the BeableBone Blue
To be sure to acquire every available frame set the CPU of the BeagleBone Blue to max speed using the command ```rc_cpu_freq```  by *[Robotics Cape SDK](http://www.strawsondesign.com/#!manual-cpu-freq)*:
```
$ rc_cpu_freq -s 1000
```




