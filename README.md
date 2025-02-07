# Driver-for-GNSS-puck-GPGGA
Collected Stationary and walking data using a GNSS Puck. Parsed the $GPGGA string for the latitude, longitude, and altitude. Converted the latitude and longitude to UTM.


## Follow the steps to perform experiment of parsing the $GPGGA message from GNSS puck

# 1. Set up the GPS puck

If you are using a virtual machine, like virtual box, you may have to:

Connect the USB.
Go to the Settings of the VM
Select the USB settings from the left pane
Add the USB device in the list from by clicking on the icon
Boot the VM with the USB device plugged in
Make sure the device is selected/captured in USB settings (click on it) before you can see it in /dev/ttyUSB* When you will connect your GPS device, you can see the device name with this terminal command.
##    
    ls –lt /dev/tty* | head
You will see a list of devices from which you have to figure out your device file identifier. Let us say it is /dev/ttyUSB2. Run this command without the GPS plugged in, the missing COM port is your GPS device. Then you need to set the read write permissions for reading the device properly.

##
    sudo chmod 666 /dev/ttyUSB2
Get minicom with:
##
    sudo apt install minicom
Configure your device’s settings in minicom:
##
    sudo chmod 666 /etc/minicom
    minicom -s
Go to serial port setup, Ctrl-a p, change the Bps/Par/Bits to “4800” and Hardware Flow Control to “No”. You will have to scroll through a series of Bps values by selecting prev or next. For example, if minicom is set to 9600 Bps then select prev to get 4800 Run minicom with :
##
    minicom
To save data to a text file, you need to use –C flag on minicom. In minicom, Ctrl-A brings up your options, including to exit. Save as gps-data.txt. When you want to stop writing to the file, press “Ctrl-a” and then “x” To check the contents of the file:

    more gps_data.txt
