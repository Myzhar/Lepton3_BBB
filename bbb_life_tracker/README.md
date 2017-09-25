# bbb_life_tracker

This demo has been realized for the *BeagleBone Blue and Lepton 3 Challenge* to show how the project "**SmarTC - Smart Thermal Camera**" works.
The idea is to detect the zone of the thermal image acquired by the Lepton3 sensor that can be associated to living beings.
This kind of information can be used for robotic applications to avoid humans and animals or to track them, according to the system behaviour.

Usage: ```bbb_life_tracker <trk_mode> <debug_ip_address> <raw_port> <res_port>  <multicast_interface> [debug_level]```

* *trk_mode*:
  **--avoid** [**-A**] / **--follow** [**-F**] -> avoid/follow elements with a temperature compatible to life
* *stream_ip_address* -> the IP address of the destination of the video stream
* *raw_port* -> the port  of the destination of the raw video stream" << endl;
* *res_port* -> the port  of the destination of the video stream with tracking result
* *multicast_interface* -> the network interface to multicast the video stream [use '' for unicast]
* debug_level [optional]:
  **0** *[default] (no debug)*- **1** *(info debug) - **2** (full debug)*

##To receive the RAW stream:

**Unicast:**

```gst-launch-1.0 udpsrc port=16000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! queue ! videoscale ! video/x-raw,width=320,height=240 ! autovideosink```

**Multicast:**

```gst-launch-1.0 udpsrc address=239.0.0.115 port=16000 multicast-iface="wlan0" ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! queue ! videoscale ! video/x-raw,width=320,height=240 ! autovideosink```

##To receive the DEBUG/INFO stream:

**Unicast:**
```gst-launch-1.0 udpsrc port=15000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! queue ! videoscale ! video/x-raw,width=640,height=480 ! autovideosink```

**Multicast:**

```gst-launch-1.0 udpsrc address=239.0.0.115 port=15000 multicast-iface="wlan0" ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! queue ! videoscale ! video/x-raw,width=320,height=240 ! autovideosink```
