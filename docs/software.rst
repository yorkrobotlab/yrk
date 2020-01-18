.. include global.rst
.. YRK User Guide: Software Setup


**************
Software Setup
**************

This user guide covers the software setup of the **YRK**.  It assumes that the micro-SD
card in the Raspberry Pi is using the pre-built **Raspbian** installation (*YRK Raspbian*)
created for the York Robotics Kit, that includes the **ROS Melodic**,  **OpenCV 4** and the
Python 3 virtual environment with all the prerequisite packages installed, along with a clone
of the **Git** repository available at::

   https://github.com/yorkrobotlab/yrk


The detailed software setup procedure followed to create the image is available in a different
document.  In the default image, the username is **pi** and the password is **robotlab**.

YRK Raspbian
------------

This document is written for **YRK Raspbian Build 16/01/2020**.  This build of Raspbian contains
the following software installations:

* Raspbian Version: Buster *(Raspbian GNU/Linux 10)*.  Output of ``lsb_release -a``

* Kernel: Linux 4.19.75-v71+ #1270 Sep 24 2019 armv71.  Output of ``uname -a``

* OpenCV: 4.1.1  Output of ``cv2.__version__`` in Python

* ROS: Melodic  Output of ``rosversion -d``

* Arduino:  1.8.10  Output of ``arduino --version``

The **yrk** PYthon virtual environment is preinstalled with a large number of required packages.
The list of packages can be found in the ``requirements.txt`` file in the ``/home/pi/yrk`` folder,
or by using the ``pip freeze`` command.

First Run
---------

The image is preconfigured to work with the ``robotlab`` wi-fi network at York; if needed, make
changes to ``/etc/network/interfaces`` before booting *(by editing the SD card in a different
Linux system)*, or connect the Pi to a display and configure networking.  Obviously in normal use
the YRK is intended to be connected to remotely using SSH or VNC etc.

On first boot of a clean install of **YRK Raspbian Build 16/01/2020** it is recommended
to update the system.  Make sure all DIP switches are in their **OFF** **(down)** position.
From the ``/home/pi`` folder execute the following script::

  . update.sh


This will update Raspbian using ``apt update`` and ``apt upgrade``,  perform a Raspberry Pi
firmware upgrade using ``rpi-eeprom-update``, run ``fixhostname.sh`` to check the hostname
has been update to the form **rpi-XXXX** *(where XXXX is last 4 digits of MAC address)*.  It
will then update to the latest codebase for **git** using ``git pull``, and clean and rebuild
the HTML documentation using ``make clean`` and ``make html`` from the docs folder.

Once the networking is setup and the system updated, the should be able to use the YRK in its
normal operating mode.  The current release defaults to booting the X-server *(even without display
attached) and auto-login; it is easy and recommended to change to command-line only using the
``raspi-config`` tool if graphical user interface isn't needed.  Both SSH and VNC *(remote-desktop)*
are enabled by default.

The documentation *(this file!)* is built in **HTML** format in the ``/home/pi/yrk/docs/_build/html/``
path, using the **Sphinx** document generation system.


Boot Procedure
--------------

The image contains entries in the ``.bashrc`` file that set the Python virtual environment to
**yrk**.  It then looks to see if the device ``/dev/i2c_11`` exists.  If the **YRK** is connected
and working correctly the i2c multiplexer device tree should be running, enabling 8 extra i2c
busses (named ``/dev/i2c_6`` to ``/dev/i2c_13`` on the **Pi 4** and ``/dev/i2c_3`` to ``/dev/i2c_11`` on the **Pi 3**).

If the device is found, the shell scripts ``/yrk/bootscript.sh`` will be launched.  It is important
to note that both scripts are called every time a new session is started (such as every new **ssh** connection).
A *core program* is run on the first call of ``bootscript.sh`` after each reboot, described in detail in the
next section.  The *core program* writes files to an area of temporary storage at `/mnt/ramdisk/`.  The
**DIP** switches at the bottom of the **YRK** determine the operation mode at boot-up, described in more detail in the next section.


The default working directory is::

   /home/pi/yrk


To kill Python processes, erase the ramdisk and restart ``bootscript.sh`` quickly use the script::

   . rerun


Core Program
------------

The core program :mod:`yrk.core` performs certain core functions that aim to improve usability
and reliability of robot controllers.  This include monitoring battery, temperature and fault
conditions, and monitoring the user switches.  It also provides the functionality to control
the *ROS* service, a web service and a demo program.  This functionality is provided through the
4-way **DIP** switch at the bottom of the kit.

* Switch 0 (*marked as 1 on the switch itself*) determines if the core program should be run on boot.
  If disabled, ``core.py`` will not be run.  This is often useful for testing but user needs to
  remember to keep check on battery and temperature.
* Switch 1 enables the ROS service using ROS launch.  If the switch is disabled after ROS has been
  launched the process will be killed, allowing a relaunch.
* Switch 2 enables the web service.  This enables a **Flask** webserver running a **Dash** site with **DAQ**
  components and this manual.  By default at ``localhost:8080``.
* Switch 3 enables the demo program.  [To do...]
