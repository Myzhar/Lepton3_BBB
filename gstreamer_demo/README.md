# gstreamer_demo

A demo that illustrates how to stream the data from Lepton3 over UDP using GStreamer

Usage: ```gstreamer_demo "dest_address" "port" <debug level>```

dest_address - port: IP Address and port of the destination of the video source

Debug level:
* 1: info messages
* 2: full debug messages

To receive the stream:

```$ gst-launch-1.0 udpsrc address=239.0.0.115 port=16000 multicast-iface="wlan0" ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink```
