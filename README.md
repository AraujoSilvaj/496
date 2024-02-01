_**udev: overview**_

udev allows a Linux system to use consistent names for devices such as removable drives and printers, which in turn allows users to experience predictable behavior when devices are added or removed from the system. 
Muliple devices set(GPS and VESC)

How to setting up the udev rule for detect the multiple usb devices connected to Linux system ?
1. Firstly, to open the location the udev-rules located locations
   
   Terminal:
   $ cd /etc/udev/rules.d/
   
3. Next, in that directory we can now copy the **udev.rules** file in the github directory into the above location.
   When you type **ls** in your **/etc/udev/rules.d/** directory is should have the **udev.rules** file

4. We can then save the created udev rule file by running reload and restart commands.

    Terminal:
    
    $ sudo service udev reload
    $ sudo service udev restart

5. Finally, plug in the all external devices(GPS,VESC) to USB port of system.

Terminal:

$ ls /dev/tty*

You should be able to see the /dev/ttyGPS and /dev/ttyVESC set.
