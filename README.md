IMU-3DM-GX4-25
==============

This repo is to help with a simple start in the LORD MicroStrain 3DM-GX4-25&trade;

For Linux and using an RS-232 connection, the test files can be run after editing the Makefile
to comment out the USB_CONNECTED variable. Once this is commented out, the device will look on ttyS ports
instead of ttyACM. 

Then from the Linux Examples folder, you can run the example by entering

```
./GX4_25_TEST 0 115200
```

then the test will begin to output depending on which port your device is connected to.

###DEPENDENCIES
[LORD]: http://www.microstrain.com/inertial/3dm-gx4-25
This code was downloaded from the (LORD MicroStrain site)[LORD] under the "MIP™ C Code Sample for Windows and Linux Version 1.1" or whatever the newest version is

Requires Zip and a gcc compiler.
