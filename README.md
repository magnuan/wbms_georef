# WBMS georef #

### Lots of bla bla here ###

Yada Yada

![](example_picture.png)


## Documentation ##

[Stable Version](https://github.com/magnuan/wbms_georef/tree/master/doc)


## Installation ##

### Windows ###
Work in progress...

Go to the [**releases** page](https://github.com/magnuan/wbms_georef/releases).   Download a zip file with "win" in its name, unzip it, and run wbms_georef.exe from a command window.

You can also build it yourself from source with MSVC

### Linux - Using git clone (recommended) ###

First you will need to install some software development packages using different commands depending on your flavor of Linux.
In most cases, the first few  will already be there and the package installer will tell you that installation is not necessary.

On Ubuntu / Raspbian / Raspberry Pi OS:

    sudo apt-get install git
    sudo apt-get install gcc
    sudo apt-get install make
    sudo apt-get install cmake

Then on any flavor of Linux:

	cd ~
	git clone https://www.github.com/magnuan/wbms_georef
	cd wbms_georef
	mkdir build && cd build
	cmake ..
	make -j4
	sudo make install
	make install-conf


