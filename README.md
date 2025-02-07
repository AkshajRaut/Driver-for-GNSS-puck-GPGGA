# Driver-for-GNSS-puck-GPGGA
Collected Stationary and walking data using a GNSS Puck. Parsed the $GPGGA string for the latitude, longitude, and altitude. Converted the latitude and longitude to UTM.


### Follow the steps to perform experiment of parsing the $GPGGA message from GNSS puck

## 1. Set up the GPS puck

If you are using a virtual machine, like virtual box, you may have to:

Connect the USB.
Go to the Settings of the VM
Select the USB settings from the left pane
Add the USB device in the list from by clicking on the icon
Boot the VM with the USB device plugged in
Make sure the device is selected/captured in USB settings (click on it) before you can see it in /dev/ttyUSB* When you will connect your GPS device, you can see the device name with this terminal command.   
    
    ls –lt /dev/tty* | head
You will see a list of devices from which you have to figure out your device file identifier. Let us say it is /dev/ttyUSB2. Run this command without the GPS plugged in, the missing COM port is your GPS device. Then you need to set the read write permissions for reading the device properly.

    sudo chmod 666 /dev/ttyUSB2
Get minicom with:

    sudo apt install minicom
Configure your device’s settings in minicom:

    sudo chmod 666 /etc/minicom
    minicom -s
Go to serial port setup, Ctrl-a p, change the Bps/Par/Bits to “4800” and Hardware Flow Control to “No”. You will have to scroll through a series of Bps values by selecting prev or next. For example, if minicom is set to 9600 Bps then select prev to get 4800 Run minicom with :

    minicom
To save data to a text file, you need to use –C flag on minicom. In minicom, Ctrl-A brings up your options, including to exit. Save as gps-data.txt. When you want to stop writing to the file, press “Ctrl-a” and then “x” To check the contents of the file:

    more gps_data.txt
##


## 2. Write a Device Driver for GNSS puck to parse $GPGGA

The GNSS puck provides several differently formatted messages. We will focus our attention on the messages that are formatted according to the $GPGGA format.

1. Ensure you can read the data from the puck (If you do not have the puck, you can use minicom as a virtual puck & the same driver should work with the real puck)
2. Parse the $GPGGA string for the latitude, longitude, and altitude. We have provided an example device driver for a depth sensor in the appendix section so that you can use that as a template
3. Convert the latitude & longitude to UTM.
4. Define a custom ROS message (called GPSmsg.msg) with a header, latitude, longitude, altitude, utm_easting, utm_northing, zone, letter as fields.
- Please match the naming & capitalization
-  Ensure correct data types.
- The Header is supposed to be a ROS Header data type. This is very important especially when you start working with tfs and do sensor fusion in ROS
- The Latitude & Longitude are supposed to be signed floats.
- The ROS Header should contain the GPGGA time stamp & not your system time (as it may be out of sync which could cause problems in a real-world system).
- The frame_ID should be a constant “GPS1_Frame” since our publisher is giving us data from the solo GPS sensor we gave you.
5. Your ROS node should then publish this custom ROS message over a topic called /gps
6. You now have a working driver, let’s make it more modular. Name this GPS driver as driver.py and add a feature to run this file with some argument. This argument will contain the path to the serial port of the GPS puck (example /dev/ttyUSB2 . Ofcourse the puck will not always be at the same port so it allows us to connect it anywhere without the script failing)
7. Even though this driver is now sufficiently modular, on a real robot we can have many sensors & launching their drivers individually can be too much work. This is where we shall use the power of ROS.
- Create a launch file called “gps_launch.py”
- This launch file should be able to take in an argument called “port” which we will specify for the puck’s port.
- Have this launch file run your gps_driver.py with the argument that was passed when it was launched.
- If you have done everything correctly, run the following command & you should
get the same results you were getting before (but with a single command!)

      ros2 launch gps_driver gps_launch.py port:=/dev/pts/6 #Or basically any ttyUSB*
##

## 3. Go outside and collect data
1. Stationary data outdoors: In a new rosbag recording, go outside and collect 10 minutes ofdata at one spot. Name this rosbag “stationary_data”
2. Walk in a straight line outdoors: In a new rosbag recording, walk in a straight line for a few hundred meters. Name this rosbag “walking_data”
##

## 4. Data Analysis
Read the rosbag data into matlab/python.

1. Analyze stationary data
2. Analyze straight line walk data
