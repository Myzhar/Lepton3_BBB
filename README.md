# Lepton3_BBB
A grabber for BeagleBone Blue to acquire thermal images from Lepton3 sensor

## SPI Buffer size
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

## Compile
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```
## Full performances
To be sure to acquire every available frame set the CPU of the BeagleBone Blue to max speed using the command ```rc_cpu_freq```  by *[Robotics Cape SDK](http://www.strawsondesign.com/#!manual-cpu-freq)*:
```
$ rc_cpu_freq -s 1000
```




