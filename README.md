# AutoDrone
Repository for the Autonomous Drone Code

For this to work first you need to install OPENCV 2.3 on your Raspberry Pi.
http://eduardofv.com/2012/05/22/installing-opencv-on-the-raspberry-pi/

You will also need the WiringPi library http://wiringpi.com/download-and-install/

After that, you can compile the code "objectTrack" into your Raspberry Pi.
The DroneControl code goes into an Arduino Uno, and it is wired to the Arduino Crius, which will control the Drone.
The receptor is connected to the Arduino Uno, and the connection between the Raspberry Pi and the Arduino Uno is made using I2C.

The diagrams for the connections will be posted soon.
