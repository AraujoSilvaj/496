# This is a AVC project

## **udev: overview**

udev allows a Linux system to use consistent names for devices such as removable drives and printers, which in turn allows users to experience predictable behavior when devices are added or removed from the system. 
Muliple devices set(GPS and VESC)

How to setting up the udev rule for detect the multiple usb devices connected to Linux system ?
1. Firstly, to open the location the udev-rules located locations
   
   Terminal:
   
   **_$ cd /etc/udev/rules.d/_**
   
3. Next, in that directory we can now copy the **udev.rules** file in the github directory into the above location.
   
   **OR**

   You could make a udev.rules file in the directory and copy the script from the **udev.rules** file in the github directory
   

   Terminal:
   
   **_$ sudo nano udev.rules_**

   **Note:** The above command creates and opens a file called udev.rules inside /etc/udev/rules.d folder.
   
   
   When you type **ls** in your **/etc/udev/rules.d/** directory it should have the **udev.rules** file
   

5. We can then save the created udev rule file by running reload and restart commands.

    Terminal:
    
    **_$ sudo service udev reload_**
   
   **_$ sudo service udev restart_**

6. Finally, plug in the all external devices(GPS,VESC) to USB port of system.

   Terminal:
   
   _$ ls /dev/tty*_
   
   You should be able to see the **/dev/ttyGPS** and **/dev/ttyVESC** set.
