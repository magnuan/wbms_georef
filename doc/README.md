# WBMS georef #

## Introduction ##
**wbms_georef** is a command line georeferencer primarily designed for Norbit WBMS sonar data.
It combines position and attitude data from a navigation system, with sensor data from sonar or lidar, and generates a georeferenced point cloud.
Supports ray tracing of sonar data if speed-of-sound profile file is provided.
Output is either CSV files or CloudCompare .sbf (simple binary format)
</pre>

![](example_picture.png)

## Options ##
|  Opt   |    Arg      |  Description  |
|--------|:-----------:|---------------|
| -c     |file	       |Read configuration file from specified location
| -x     |             |Generate template config file with default values (wbms_georef.conf.template)
| -i     |file         |Filename for file containing sensor and navigation data. Typically s7k.
| -s     |file         |Filename for file containing sensor data. Sonar in raw WBMS format or s7k. Lidar in raw Velodyne format.
| -p     |file         |Filename for file containing navigation data. Raw Applanix PosMV, s7k and some other CSV-formats supported.
| -w     |file         |Filename for file containing sound velocity profile.<br /> Data should be comma separated with sound velocity in [m/s] and depth in [m], one measurement per line.
| -o     |file         |Filename prefix of output file.<br />If outputting to CSV, outputfile.csv will be generated.<br /> If outputting to SBF, outputfile.sbf and outputfile.sbf.data will be generated.
| -S     |val          |Sensor data format. 1=WBMS 2=WBMS_V5 3=Velodyne 5=S7K<br /> If none is given here or in config file, wbms_georef will try to auto detect.
| -P     |val          |Navigation data format. 0=POSMV 1=XTF_NAV 2=WBM_TOOL 3=SBET 4=SIM 5=S7K<br /> If none is given here or in config file, wbms_georef will try to auto detect.
| -C     |             |Output to CSV format
## Basic usage ##

Simplest use, if you have a data file containing both sensor data and navigation.

     wbms_georef -i some_file.s7k -o some_file
     
*wbms_georef* will then:
* autodetect file format
* pick apropriate UTM zone based on navigation data
* assume sensor reference frame equal to navigation
* assume uniform sound velocity profile
* process data with permissive data filtes 
* output point clound in SBF (Cloud Compare simple binary format)

**Input file format** will by defualt be auto-detected. Current implementation of this is a bit slugish, so if you are in a hurry and have multiple files to process, it can set the format with the -S and -P option.

## Config file ##
There are just too many parameters that can be configured for simple arguments to be practical.
Instead *wbms_georef* can read configuration from a text file.
To generate a template with "default" values run:

  wbms_georef -x
 
This will generate a file named *wbms_georef.conf.template* containing default settings, and some basic information about what the different parameters does.

[Here](https://github.com/magnuan/wbms_georef/blob/main/doc/wbms_georef.conf.template) is an example of such a file for reference, but to get the latest version, please generate it with *wbms_georef* instead.

### Config file sections ###
#### SENSOR MOUNTING ####
6 parameters to define the reference frame of the sensor (sonar/lidar) relative to navigation reference frame origin.  
Translations is done in navigation reference frame **before** rotations.  
Rotations are done in the order **yaw**, **pitch**, **roll**.  

##### Coordinate system is defined: ##### 
* Right hand system
* **X** forward
* **Y** Starboard
* **Z** downwards 
* **roll** around **X**
* **pitch** around **Y** 
* **yaw** around **Z**
    
 <img src="https://github.com/magnuan/wbms_georef/blob/main/doc/cord_system.png" width="300" height="300">


##### Sonar default orientation: #####
* Scanning (primarily) in YZ-plane, vessel moving in X-direction
* Pointing downwards, azimuth=0, elevation=0  pointing in Z-direction
* Positive azimuth direction in negative roll direction, from nadir to starboard
* Positive elevation in positive pitch direction from nadir to forward

![Sonar default mounting](https://github.com/magnuan/wbms_georef/blob/main/doc/Sonar_default_mounting.png "Sonar default mounting")

##### Lidar default orientation: #####
 * Scanning (primarily) in YZ-plane, vessel moving in X-direction
 * Pointing upwards, azimuth=0, elevation=0  pointing in negative Z-direction
 * Positive azimuth direction in negative roll direction, from zenith to port
 * Positive elevation direction in negative pitch direction, from zenith to forward

![Lidar default mounting](https://github.com/magnuan/wbms_georef/blob/main/doc/Lidar_default_mounting.png "Lidar default mounting")

```bash
# Linear offsets are given in meters X=pos.forward, Y=pos.starboard, Z=pos.down
# Rotation offsets are given in degrees, Yaw=pos.turning starboard, Pitch=pos.tilting up, Roll=Rotating CW

# Sonar offset and roll relative to Navigation origo. 0,0,0 attitude is Rx facing downwards, Tx pointing aft
sensor_x_offset 0.0
sensor_y_offset 0.0
sensor_z_offset 0.0
sensor_yaw_offset 0.0
sensor_pitch_offset 0.0
sensor_roll_offset 0.0
```


[Stable Version](https://github.com/magnuan/wbms_georef/tree/master/doc)


## Installation ##

### Windows ###

Go to the [**releases** page](https://github.com/magnuan/wbms_georef/releases).   Download a zip file with "win" in its name, unzip it, and run wbms_georef.exe from a command window.

You can also build it yourself from source.  For more details see the **User Guide** in the [**doc** directory](https://github.com/magnuan/wbms_georef/tree/master/doc).

### Linux - Using git clone (recommended) ###

First you will need to install some software development packages using different commands depending on your flavor of Linux.
In most cases, the first few  will already be there and the package installer will tell you that installation is not necessary.

On Debian / Ubuntu / Raspbian / Raspberry Pi OS:

    sudo apt-get install git
    sudo apt-get install gcc
    sudo apt-get install make
    sudo apt-get install cmake

Then on any flavor of Linux:

	cd ~
	git clone https://www.github.com//magnuan/wbms_georef
	cd wbms_georef
	mkdir build && cd build
	cmake ..
	make -j4
	sudo make install

