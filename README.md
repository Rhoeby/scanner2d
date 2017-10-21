# Rhoeby Dynamics R2D LiDAR ROS Node

The Rhoeby Dynamics R2D LiDAR is a small, light, low-cost scanner that uses IR (Infra-Red) ranging technology from TeraRanger (website: http://teraranger.com). This scanner is designed to replace more expensive laser-based products and avoids the eye-safety issues associated with those devices. For more information on the scanner, please see our website: http://www.rhoeby.com.

## Table of Contents

[Specifications](https://github.com/Rhoeby/scanner2d#specifications)

[Quick instructions](https://github.com/Rhoeby/scanner2d#quick-instructions)

[Detailed instructions](https://github.com/Rhoeby/scanner2d#detailed-instructions)

[Notes](https://github.com/Rhoeby/scanner2d#notes)

[Troubleshooting](https://github.com/Rhoeby/scanner2d#troubleshooting)

## Specifications

  - Scan rate: 1 - 5 Hz, user-configurable
  - Scan range: 360 degrees, over 5 meters
  - Angular resolution (minimum): 0.36 degrees (@ 1 Hz scan rate)
  - Sample density: 250 - 1000 Samples/scan, user-configurable
  - Range accuracy: +/- 5-10 cm
  - Size: 75 x 44 x 44 mm (with base enclosure)
  - IR-based sensor
  - Interfaces: Micro-USB, UART

## Quick instructions

1. install/build scanner2d ROS node

2. connect scanner via USB

3. run static tf publisher

    rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_laser map 1000 &

4. run scanner2d node

    rosrun scanner2d scanner2d _port_name:="/dev/ttyACM0"

5. view data in rviz (topic: '/laser_data', frame_id: 'base_laser')


## Detailed instructions

1. install/build scanner2d ROS node

  - download code from https://github.com/Rhoeby/scanner2d
  - place code in sub-directory of catkin workspace
  - do 'catkin_make'
  - do 'source devel/setup.bash'

2. connect scanner via USB

  - plug in scanner
  - type 'lsusb', you should see something like: "Bus 002 Device 074: ID 0483:5740 STMicroelectronics"
  - confirm sensor blue LED is flashing
  - you could verify *binary* data flow from the scanner, type: 'cat /dev/ttyACM0' (use Ctrl-C to exit)

3. run roscore

    roscore &

4. run static tf publisher

    rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_laser map 1000 &

 This allows the scan to be displayed in RViz by providing a transform from the frame of the scanner to the map.

5. run ROS node as shown here (it's using ttyS3, as an example):

    rosrun scanner2d scanner2d _port_name:="/dev/ttyS3" _scan_rate:=4

 The default port name is "/dev/ttyACM0". If you are using a different port, you'll need to specify it on the command line.

6. Confirm scanner starts and speeds up to 240 rpm (4 rotations / sec)

7. run RViz

    rosrun rviz rviz

9. observe scanner plot data in rviz

  - in rviz, select "Add"
  - select "By topic" tab
  - you should see '/laser_data' as a topic
  - select LaserScan which appears under the topic
  - hit OK

 The LaserScan should now be presented in rviz. If you can see it, congratulations! If it does not work, try the Troubleshooting guide below.

## Notes

When the ROS node is started, the scanner gets reset. As part of the reset, the scanner starts at a slow speed, then it speeds up to the operational scan rate (whatever is set on the command line). This is required to get the scanner initialized and into a known state prior to starting normal operation.

The Rhoeby R2D LiDAR provides completely **unfiltered scan results**, when used with it's default setting. As a result, users may find that there is significant noise in the scan data, as well as "veiling" effects around edges, and other such artifacts. These effects can be eliminated to a great extent by enabling "sample rejection". This can be done when invoking the ROS driver, as follows:

    rosrun scanner2d scanner2d _sample_rejection:=1

Doing the above will result in similar readings to that given by other devices. When doing lab tests, and especially if doing side-by-side comparisons with laser-based devices, it is recommended that users enable sample rejection, as described above.

It's possible to run the scanner at different speeds. From the command-line, you could set the paramter 'scan_rate'. 

    rosrun scanner2d scanner2d _scan_rate:=3

Actually, there are several parameters that can be set. Here is a sample command line:

    rosrun scanner2d scanner2d _port_name:="/dev/ttyACM0" _scan_rate:=3 _sample_rejection:=0 _samples_per_scan:=333 _min_angle:=0 _max_angle:=360

These settings are added to the ROS parameter server when the driver is started.

## Troubleshooting

[Scanner fails to respond when ROS node is started](https://github.com/Rhoeby/scanner2d#solution)

[When starting the scanner2d node, it reports: "[ERROR] [1427572321.434331608]: /dev/ttyACM0 open failed!
" or similar](https://github.com/Rhoeby/scanner2d#solution-1)

[RViz does not display the scan](https://github.com/Rhoeby/scanner2d#solution-2)

[My scanner is showing a lot of noise in the scan data](https://github.com/Rhoeby/scanner2d#solution-3)

[Scanner does not enumerate (does not show up as COM port) under Windows](https://github.com/Rhoeby/scanner2d#solution-4)

[When I connect the Mini-USB port to my Windows PC, the driver is not recognized (Vista/7/8/10)](https://github.com/Rhoeby/scanner2d#solution-5)

### Scanner fails to respond when ROS node is started (shows message "Waiting for reset CLEAR to complete...")

### Solution:

Run the following

    cat /dev/ttyACM0 (or whatever device your scanner is on)

Is binary data presented?

If yes, restart the scanner ROS node, and try again.

If no, the problem is likely to be connectivity between the scanner and your Linux system... check you have it connected correctly and have specified the correct device (eg. /dev/ttyS2). Try unplug/plug.

### When starting the scanner2d node, it reports: "[ERROR] [1427572321.434331608]: /dev/ttyACM0 open failed!" or similar

### Solution:

Make sure you have permission to access '/dev/ttyACM0' (or other)

### RViz does not display the scan

### Solution:

With the scanner running do the following:

    open another terminal
    run 'rostopic echo /laser_data'

Do you see any data?

If yes, then the problem is with RViz and I would suggest to check for any error in RViz. If you "Add by Topic", does the 'laser_data' topic show up?

If there is no data, it's likely a problem with the scanner. Do the following:

    shutdown rviz (using Ctrl-C)
    shutdown the scanner2d node (using Ctrl-C)
    shutdown the static_transform_publisher

    shutdown roscore

    unplug scanner

    plug scanner back in

    restart roscore
    restart static_transform_publisher
    restart scanner2d node
    restart rviz

If that fails, try running 'rosrun rqt_console rqt_console' and set scanner2d to Debug (with scanner and driver running). Do you see status messages?

### My scanner is showing a lot of noise in the scan data

### Solution:

The Rhoeby R2D LiDAR provides completely **unfiltered scan results**, when used with it's default setting. As a result, users may find that there is significant noise in the scan data, as well as "veiling" effects around edges, and other such artifacts. These effects can be eliminated to a great extent by enabling "sample rejection". This can be done when invoking the ROS driver, as follows:

    rosrun scanner2d scanner2d _sample_rejection:=1

Doing the above will result in similar readings to that given by other devices. When doing lab tests, and especially if doing side-by-side comparisons with laser-based devices, it is recommended that users enable sample rejection, as described above.

### Scanner does not enumerate (does not show up as COM port) under Windows

### Solution:

    Unplug scanner from USB
    Select "scan for hardware changes" (in Windows Device Manager)
    Plug in scanner

### When I connect the Mini-USB port to my Windows PC, the driver is not recognized (Vista/7/8/10)

### Solution:

Please try: 

  - a different cable (some customers have had bad cables)
  - a different USB port
  - reboot your PC

For Windows 7/8, you could try installing this device driver:

  http://www.st.com/web/en/catalog/tools/PF257938

(you may need to reboot after install)

If that fails to get the LiDAR recognized, please try: 

  - a different PC (preferably with different OS version)

(Note: we have tested the device with Windows 7, 8 and 10. Also, tested with various Linux versions)
