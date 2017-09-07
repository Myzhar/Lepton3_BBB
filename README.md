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
at the end of the line, like this:
```
cmdline=coherent_pool=1M net.ifnames=0 quiet cape_universal=enable spidev.bufsiz=131072
```
Save, exit and reboot

## Compile
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```




