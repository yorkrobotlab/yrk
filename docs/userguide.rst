.. include global.rst
.. YRK User Guide


**********
User Guide
**********

Overview
--------

The **York Robotics Kit** is a platform design to allow the Raspberry Pi family of microcomputers
to control a wide variety of robotics hardware with the minimum extra electronics assembly.
It comprises a pair of PCBs which attach above a Raspberry Pi SBC [single-board computer].
This document describes the initial version of the York Robotics Kit, comprising the YRL039 Power Supply Board and the YRL040 YRK Board, which are primarily designed to be used with the Raspberry Pi 4 B and Raspberry Pi 3 B(+) series of computers.
This is still very much a prototype and development design and should be regarded as such!


.. figure:: /images/pinout.jpg
    :width: 600px
    :height: 772px
    :alt: Pin-out and wiring diagram for York Robotics Kit

    Pin-out and wiring diagram for York Robotics Kit


Construction
------------

.. sidebar:: YRL040 PCB Variants

   **1.0** *[Sep 2019]* Original version as seen in pictures, models and diagrams

   **1.1** *[Dec 2019]* Navigation switch changed to surface-mount model, support for 3.5mm screw terminals for motors added.

This document assumes PCBs have been manufactured and all components assembled **(see separate documentation for PCB construction)**
The schematic diagrams and layout documents are included as Appendices.
Both PCBs are two-layer boards and designed to be both relatively cheap to manufacture and relatively easy to assemble.


YRL039 Power Supply
^^^^^^^^^^^^^^^^^^^


.. figure:: /images/yrl039.jpg
    :width: 300px
    :height: 369px
    :alt: 3D rendering of the top of the YRL039 power supply

    YRL039 Power Supply



The YRL039 Power Supply Board as designed to serve six main functions:

* Take a battery or DC input and convert into stable, separate 5V supplies.  Each of the 2 5V supplies is rated to provide at least **2.5A** current.  One supplies the Raspberry Pi *(including USB peripherals)*, the other provides the **5V_AUX** rail for most other components.

* Provide an interface between Pi GPIO pins and respective pins on YRL040 PCB.  It was desired to rotate and reduce in size the 40-pin GPIO header on the Pi to allow more space for edge-mount connectors on either side of the YRL040 PCB **(for motors, sensors and servos etc)**.

* Provide hardware monitoring of temperature, input voltage, output voltages and output currents.

* Provide a software on-off power button with forced shutdown option

* Provide an 35mm fan based active cooling for the Pi microcontroller, the power supplies and the YRL040 mounted above.

* Generate simple audio tones using a Piezo buzzer

All the components except for the fan are mounted to the top-side of the PCB.  The **TPS82130** `TI 3A Step-Down Converter Module <http://www.ti.com/lit/ds/symlink/tps82130.pdf>`_ is core components of each of the two 5V power supplies.


Programming the YRL039
++++++++++++++++++++++

The YRL039 uses at ATMega328P microcontroller to control the soft-power switch, make audio tones and report voltage, current and temperature readings using the I2C
bus.  The standard code is found in the ``atmega_code/yrl039_arduino`` subfolder of the GIT repository; if special functionality is required, such as lowering the
battery-low cutoff voltage or reducing the temperature at which the fan operates, it may be necessary to reprogram the microcontroller.  This is easiest to do with
the **YRL030** *FTDI Interface and Programming Board* that contains the 8-pin, 1.25mm pitch connector used for programming and serial data; the **YRL030** can be used
for both flashing the bootloader to the microcontroller using the **ICSP** interface and also uploading code and debugging data using a USB serial interface.


YRL040 Main PCB
^^^^^^^^^^^^^^^

The main YRL040 PCB has been designed such that the vast majority of electronic components are surface mounted on the underside of the board.


.. figure:: /images/yrl040.jpg
    :width: 560px
    :height: 399px
    :alt: Bottom and top 3D renderings of the top of the YRL040 PCB

    YRL040 Main PCB *(Bottom and Top Views)*


The board contains the following interfaces:

* 8-way I2C switch providing 4 user busses at 3.3V, a 5V bus used by on board PWM driver and Arduino, a bus
  design for use with an OLED display module, a bus for communication with the YRL039 PSU and a bus for all
  other internal connections.

* 4 I2C H-Bridge Motor Drivers based on the TI **DRV8830** IC.

* 16-channel PWM Driver suitable for use with analogue servo motors

* 16-channel GPIO expander providing 5-way navigation switch, 4-way DIP switch, 2 push-button switches and LEDs

* Addition 16-channel GPIO expander providing motor driver fault monitoring, a 5V and a 12V switched output,
  a kill-switch and 8 user GPIO pins

* 8-channel, 8-bit ADC with 6 ports optimised for use with Sharp analogue distance sensors, a channel connected
  to a potentiometer and a spare channel.

* A mono audio amplifier connected to a PWM audio channel on the Raspberry Pi

* An I2S mono microphone module

* An Arduino compatible ATMega328P microcontroller for use with wheel encoding, digital servo motors and other
  tasks.


Assembly
^^^^^^^^

.. figure:: /images/exploded_view.jpg
    :width: 600px
    :height: 570px
    :alt: Exploded view of York Robotics Kit assembly

    Exploded view of YRK showing 11mm standoffs and M2.5 screws


The YRL039 attaches above the Raspberry Pi PCB using the 4 x 11mm length, M2.5 diameter standoffs.  Another layer of 4 x 11mm standoffs is used to attach the *YRL040*
PCB above the **YRL039** power supply PCB.  It is recommended that *M:F* **(male one end, female other)** standoffs are used in the upper layer, with the male threads pointing downwards through the *YRL040* PCB, then M:M standoffs **(sometimes called spacers)** below on the bottom layer.


The assembly can be mounted inside a further case, described below, or can be mounted directly onto a chassis or further standoffs.  Consideration of the airflow path should be taken, particularly when fully enclosed inside a robot.  The Raspberry Pi alone can generate a significant amount of heat and rapidly reaches a point at which it will throttle clock speed if it is not adequately cooled.

Case
^^^^

.. figure:: /images/case.jpg
    :width: 600px
    :height: 467px
    :alt: YRK mounted within case

    York Robotics Kit mounted in Polyjet case


A case designed *specifically* for the **Raspberry Pi 4B** series of computer, the **YRL039** and the **YRL040** PCBs has been designed.  This particular design is intended to be printed on a Polyjet class of 3D printer, with very fine tolerancs and gaps.  A more general design for FDM could easily be implemented, but is not essential.  If designed a new housing [or placing assembly within a robot chassis design etc], consider airflow route carefully.  Pi 4 devices will generate substantial heat and the fan needs some route to direct air across the Pi 4 CPU, but also ideally the power supply elements on the top side of YRL039 and also components on the underside of the YRL040 PCB.  Those components likely to dissipate the most heat [motor drivers, PWM driver, amplifier and active outputs] are all towards the lower half of the PCB, which should receive forced convection from the fan.

Connecting Hardware
-------------------

Power Supply
^^^^^^^^^^^^

The YRK can be powered by any standard 12V power supply with a 2.1mm DC jack (centre positive) and 3A or greater current rating.  Using a tethered power supply such as this is strongly
recommended to be used as much as possible to preserve battery life if motion of the robot is not required.


Battery
^^^^^^^

The YRK is primarily inteded to be run using a 3-cell Lithium-Polymer battery pack, connected using the industry-standard **JST-XH** 4-pin 0.1" pitch connector
used as the balance charging connector.  These connectors and cables are typically rated for 2A current which is enough for normal use of the YRK [which has a **2.5A** input Polyfuse].  Where high
currents are needed, such as powering multi servo motors, the high current cable from the battery should be soldered *(via a self-made adapter)* to the appropriate power points on the **YRL040** PCB.


The **YRL039** power supply board has a low-voltage dropout and should work effectively from voltages as low as **5.5V** to a **17V** high, enabling 2-cell and 4-cell operation Li-Po (and a range
of other lead-acid, Ni-Mh, NiCad, LiFe and other rechargeable battery technologies, provided they can provide around 5A peak current at 5V).  Relevant changes to the ``yrk.settings`` file should
be made to reflect the battery used if not the default 3-cell configuration.

Only the outer-two connections of the **JST-XH** connector are used.  It is also possible to use the 2.1mm DC jack as the input source.  Always disconnect the battery after use and charge using a
suitable charger (in a fire-safe bag if using a Li-Po battery pack).  The power supply board is not presently optimised for ultra low-current and the residual current draw (from the ATMega microcontroller)
will discharge the battery even when beyond a damagingly low value.  General convention suggest a per-cell voltage of 3.0V is the absolute minimum a Li-Po battery should be allowed to discharge to *(ie 9V
for 11.1V 3-cell battery) before permanent damage is likely to be done.  The Arduino code on the **YRL039** can be configured to automatically switch off power supplies below a critical low voltage but this
needs to be correctly configured for the battery technology used.  If the ``core.py`` software is *not* run the user must implement some method of periodically monitoring the battery voltage and provide
suitable user warnings when low levels are reached, otherwise batteries can rapidly deteriorate.

The YRK connected to a Raspberry Pi 4 will typically consume approximately 800mA on the 5V rails at idle, most the current powering the Raspberry Pi and a smaller amount of the **5V_AUX** rail (mostly
powering the fan and LEDs).  Motors and sensors will add significantly to this load.  This relatively high idle load means that battery capacity should be at least 1000mAH, and this capacity would
provide at best around 1 hours use *(and significantly less in high load cases)*.  The use of a lower-power Raspberry Pi, such as the model 3A, might be considered when long battery life on small batteries
is desired, at the expense of memory, processing capability and expandability.



DC Motors
^^^^^^^^^

There are four serial H-Bridge motor drivers, based on the **DRV8830** `TI Motor Driver <http://www.ti.com/lit/ds/symlink/drv8830.pdf>`_.
The PCB design limits each motor driver to approximately **800mA** current, powered from the *5V_AUX* supply.  Having all four motors drawing this peak current
for sustained periods will exceed the rating of the power supply.  This current limit *(and voltage rating)* does restrict the motor driver to using small motors,
such as the widely-available 3mm shafted **micro-metal gear motors**.  Before using a different size of motor it is recommend to check *(such as by using a bench
PSU)* what the stall and no-load currents at **5V**.

The holes on the unpopulated PCB allow the motors to be connected to either **Wago** push-fit terminals or **(on PCB version 1.1)** 3.5mm pitch screw terminals.
With either connector, a remaining pair of holes will be accessible on the PCB should a direct soldered lead be required.


Servos
^^^^^^

The YRK can control both standard analogue servo motors (8 directly attachable, 8 further channels available via breakout), and digital servo motors via an
Arduino-based software interface.


Analogue Servos
+++++++++++++++

Analogue servos are operated using a **PCA9685** `I2C LED driver IC <https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf>`_.
Whilst primarily designed to allow I2C brightness control of up to 16 LEDs,  it can effectively work as a analogue servo controller.
Analogue servos typically operate with a **20mS** period width *(50Hz PWM frequency)*, and expect a pulse width in the **1ms - 2ms** range [with **1.5ms** being the middle point of the servo rotation].
The **PCA9685** lets us fix the PWM frequency for all outputs and effectively becomes an I2C servo controller.

There are 16 available outputs on the YRL040 PCB, located in the middle-top of the PCB.
Eight of these are available as full 3-pin outputs, where DC power (**+** and **GND**) can be supplied to the servo.
Most analogue servos come hardwired with a three-pin 0.1” pitch socket attached at the wire tail.
Different colour schemes are used for the wiring, and it is important to be careful checking the orientation of the plug;
as a general rule the lightest colour will the control signal (top side of connector) and the darkest will be ground.

==========  =======   ======  ======  ===============
Pin Number  Signal    Futaba  JR      Hitec
==========  =======   ======  ======  ===============
3 [Top]     Control   White   Orange  Yellow or White
2           V+        Red     Red     Red or Brown
1 [Bottom]  Ground    Black   Brown   Black
==========  =======   ======  ======  ===============


These 8 complete connections are split into two banks of four.  Each of these banks can be supplied with DC either by the internal 5V supply **(by using a 2mm jumper)**, or to an external positive input, by soldering a suitable cable onto the hole on the board.  If the internal supply is used, the total current for each bank **must not excede 1A** (a pair of 0603 fuses are included on the board .  This is due to the overall current limitations on the board.  For this reason it is strongly recommended to only use very small, low-current servos, and to spread load across both banks, if using the internal supply.

Another 8 PWM outputs are available just below the primary 8, but these cannot be used directly with a 3-pin connector.  In situations where a large number of servos are required simultaneously, a small break-out board allowing direct power connection would be a sensible option.
The circuit is the same as used on the `Adafruit 16-channel PWM servo driver <https://learn.adafruit.com/16-channel-pwm-servo-driver>`_.

Code for the analogue servo control is in the :mod:`yrk.pwm` module.  Examples of the use of the PWM
driver to control servos can be found in :mod:`examples.console`.

Digital Servos
++++++++++++++

The York Robotics Kit is designed to support digital servos from the **(Dynamix AX- and MX- series)** via code on the Arduino microcontroller.

To do: This section and code not completed yet!


Analogue Inputs
^^^^^^^^^^^^^^^

The YRK includes an I2C based, 8-channel, 8-bit analogue to digital converter IC.  Whilst this can be used for anything
requiring analogue inputs, it is primarily intended for use with analogue distance
sensors manufactured by Sharp, in particular the **2Y0A21** and **2Y0A41** models.  The reference voltage is set to **2.5V**,
meaning the returned value is approximately equal to the voltage mulitplied by 100.

.. figure:: /images/adc.jpg
    :width: 580px
    :height: 130px
    :alt: Pin-out for channels 0-5 and channel 7 of the analog to digital converter

    Analogue input channels 0-5 *(JST PH sockets)* and channel 7


Cables
++++++

.. figure:: /images/sharp.png
    :width: 600px
    :height: 182px
    :alt: Wiring diagram of Sharp Distance Sensors

    Wiring diagram of Sharp Distance Sensors


The Sharp distance sensors use a 3-pin JST PH series connection **(note the newest models use a JCTC connector instead of
a JST)**.  6 matching JST-PH connections are available on the York Robotics Kit, each providing the analogue-input and 5V
power supply required by the sensor.  A suitable complete pre-made harness has not been sourced, but it is possible to buy
pre-crimped leads from JST which make creating harnesses quick and simple *(if expensive)*.

===============   ============  ===================  ======================
JST Part Number   Farnell Part  Description          Unit Price *[per 100]*
===============   ============  ===================  ======================
01SPHSPH-26L150   2065431       150mm PH-PH Lead     0.416
01SPHSPH-26L300   2065432       300mm PH-PH Lead     0.439
PHR-3             3616198       3-pin PH Receptacle  0.032
===============   ============  ===================  ======================

To assemble the harness, place one receptacle face-up and the other face-down then connect top-to-top, middle-to-middle and bottom-to-bottom,
as seen in the photograph below.

.. figure:: /images/jstcable.jpg
    :width: 600px
    :height: 62px
    :alt: Assembly of a JST PH cable for use with Sharp Distance Sensors

    Assembly of first wire in JST PH cable for use with Sharp Distance Sensors

Datasheets
++++++++++

`Sharp 2Y0A21 [10-80cm] <https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf>`_

`Sharp 2Y0A41 [4-40cm] <https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf>`_


Potentiometer
+++++++++++++

Channel 6 of the ADC is connected to a potentiometer *(variable resistor)* at the top-left of the PCB.  As the pot'
is rotated clock-wise from left to right the ADC output value will **decrease** from 255 to 0.

Other Inputs
++++++++++++

Channel 7 of the ADC is routed to the left pin of a 2mm pitch pin-header below the channel 5 connector.  There is
also the potential to use any of the 8 available analogue inputs on the ATMega microcontroller which offers 10-bit
resolution *(see section on Arduino below)*.


Arduino
^^^^^^^

The YRK includes a **ATMega328P** microcontroller, running at **5V** and connected to both an
FTDI serial to USB interface and to the I2C switch (on switch port 5, which is ``/dev/i2c_11`` on Pi 4).
The microcontroller is effectively a clone of an Arduino Nano board (albeit with a different pin layout).

.. figure:: /images/arduino.jpg
    :width: 600px
    :height: 333px
    :alt: Pinout for ATMega microcontroller expansion

    Pin-out for the ATMega microcontroller *(Arduino nano clone)*


Switched Outputs
^^^^^^^^^^^^^^^^

The board contains a pair of **FET** driven switched outputs which can be used when it is necessary to turn
on simple switched loads.  Typical uses might be powering buzzers and sirens, LED light fittings and lamps,
beacons, solenoids and relays.  One output is connected to the **5V_AUX** supply, the other is marked as **12V** and is
connected to the battery or DC input.  Both switched outputs are protected by a 1A 0603 quick-blow fuse.  The outputs are
connected to 0.1" sockets *(preferred over header as harder to short-circuit)*.

.. figure:: /images/switched-outputs.jpg
    :width: 561px
    :height: 300px
    :alt: Switched output connectors

    Close-up view of 12V and 5V switched output connectors.
    

It is important to note that the switched outputs use low-side switching, meaning that the **+** output is connected directly
to the *(5V or battery)* supply rail but the **-** is **not** connected to ground; never use the switched outputs on loads
that require the grounds to be coupled together.  It is recommended to limit the current on the switched outputs to below 500mA
if possible.  If a higher current (or circuit with coupled ground) is needed, consider using the switched load to drive a relay
or solid-state equivalent.   Note that the actual potential difference will be a little lower than the indicated amount due to
the voltage drop across the **FET**.  Consider using a flyback diode across inductive loads (such as relays and solenoids).


Software Setup
--------------

This user guide assumes that the micro SD card in the Raspberry Pi is using the pre-built
**Raspbian** installation (*YRK Raspbian*) created for the York Robotics Kit that includes the
**ROS Melodic**,  **OpenCV 4** and the Python 3 virtual environment with all the prerequisite packages installed,
along with a clone of the **Git** repository available at::

   https://github.com/yorkrobotlab/yrk


The detailed software setup procedure followed to create the image is available in a different
document.  In the default image, the username is **pi** and the password is **robotlab**.

YRK Raspbian
^^^^^^^^^^^^

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
^^^^^^^^^

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
^^^^^^^^^^^^^^

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
^^^^^^^^^^^^

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
