.. include global.rst
.. YRK User Guide

User Guide
==========

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
+++++++++++++++++++


.. figure:: /images/yrl039.jpg
    :width: 300px
    :height: 369px
    :alt: 3D rendering of the top of the YRL039 power supply

    YRL039 Power Supply



The YRL039 Power Supply Board as designed to serve five main functions:

* Take a battery or DC input and convert into stable, separate 5V supplies.  Each of the 2 5V supplies is rated to provide at least **2.5A** current.  One supplies the Raspberry Pi *(including USB peripherals)*, the other provides the **5V_AUX** rail for most other components.

* Provide an interface between Pi GPIO pins and respective pins on YRL040 PCB.  It was desired to rotate and reduce in size the 40-pin GPIO header on the Pi to allow more space for edge-mount connectors on either side of the YRL040 PCB **(for motors, sensors and servos etc)**.

* Provide hardware monitoring of temperature, input voltage, output voltages and output currents.

* Provide a software on-off power button with forced shutdown option

* Provide an 35mm fan based active cooling for the Pi microcontroller, the power supplies and the YRL040 mounted above.


All the components except for the fan are mounted to the top-side of the PCB.  The **TPS82130** `TI 3A Step-Down Converter Module <http://www.ti.com/lit/ds/symlink/tps82130.pdf>`_ is core components of each of the two 5V power supplies.


YRL040 Main PCB
+++++++++++++++

The main YRL040 PCB has been designed such that the vast majority of electronic components are surface mounted on the underside of the board.


.. figure:: /images/yrl040.jpg
    :width: 560px
    :height: 399px
    :alt: Bottom and top 3D renderings of the top of the YRL040 PCB

    YRL040 Main PCB *(Bottom and Top Views)*



Assembly
++++++++

.. figure:: /images/exploded_view.jpg
    :width: 600px
    :height: 570px
    :alt: Exploded view of York Robotics Kit assembly

    Exploded view of YRK showing 11mm standoffs and M2.5 screws


The YRL039 attaches above the Raspberry Pi PCB using the 4 x 11mm length, M2.5 diameter standoffs.  Another layer of 4 x 11mm standoffs is used to attach the *YRL040*
PCB above the **YRL039** power supply PCB.  It is recommended that *M:F* **(male one end, female other)** standoffs are used in the upper layer, with the male threads pointing downwards through the *YRL040* PCB, then M:M standoffs **(sometimes called spacers)** below on the bottom layer.


The assembly can be mounted inside a further case, described below, or can be mounted directly onto a chassis or further standoffs.  Consideration of the airflow path should be taken, particularly when fully enclosed inside a robot.  The Raspberry Pi alone can generate a significant amount of heat and rapidly reaches a point at which it will throttle clock speed if it is not adequately cooled.

Case
++++

.. figure:: /images/case.jpg
    :width: 600px
    :height: 467px
    :alt: YRK mounted within case

    York Robotics Kit mounted in Polyjet case


A case designed *specifically* for the **Raspberry Pi 4B** series of computer, the **YRL039** and the **YRL040** PCBs has been designed.  This particular design is intended to be printed on a Polyjet class of 3D printer, with very fine tolerancs and gaps.  A more general design for FDM could easily be implemented, but is not essential.  If designed a new housing [or placing assembly within a robot chassis design etc], consider airflow route carefully.  Pi 4 devices will generate substantial heat and the fan needs some route to direct air across the Pi 4 CPU, but also ideally the power supply elements on the top side of YRL039 and also components on the underside of the YRL040 PCB.  Those components likely to dissipate the most heat [motor drivers, PWM driver, amplifier and active outputs] are all towards the lower half of the PCB, which should receive forced convection from the fan.

Connecting Hardware
-------------------

DC Motors
+++++++++

There are four serial H-Bridge motor drivers, based on the **DRV8830** `TI Motor Driver <http://www.ti.com/lit/ds/symlink/drv8830.pdf>`_.
The PCB design limits each motor driver to approximately **800mA** current, powered from the *5V_AUX* supply.  Having all four motors drawing this peak current
for sustained periods will exceed the rating of the power supply.  This current limit *(and voltage rating)* does restrict the motor driver to using small motors,
such as the widely-available 3mm shafted **micro-metal gear motors**.  Before using a different size of motor it is recommend to check *(such as by using a bench
PSU)* what the stall and no-load currents at **5V**.

The holes on the unpopulated PCB allow the motors to be connected to either **Wago** push-fit terminals or **(on PCB version 1.1)** 3.5mm pitch screw terminals.
With either connector, a remaining pair of holes will be accessible on the PCB should a direct soldered lead be required.

Servo Motors
++++++++++++

The York Robotics Kit is designed to support both standard-analogue servos and also digital servos **(Dynamix AX- and MX- series)**.

Analogue Servos
^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^

To do: This section and code not completed yet!


Software Setup
--------------

This user guide assumes that the micro SD card in the Raspberry Pi is using the pre-built
**Raspbian** installation created for the York Robotics Kit that includes the **ROS Melodic**,
**OpenCV 4** and the Python 3 virtual environment with all the prerequisite packages installed,
along with a clone of the **Git** repository available at::

   https://github.com/yorkrobotlab/yrk


The detailed software setup procedure followed to create the image is available in a different
document.  In the default image, the username is **pi** and the password is **robotlab**.


Boot Procedure
++++++++++++++

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
++++++++++++

The core program :mod:`yrk.yrk-core` performs certain core functions that aim to improve usability
and reliability of robot controllers.  This include monitoring battery, temperature and fault
conditions, and monitoring the user switches.  It also provides the functionality to control
the *ROS* service, a web service and a demo program.  This functionality is provided through the
4-way **DIP** switch at the bottom of the kit.

* Switch 0 (*marked as 1 on the switch itself*) determines if the core program should be run on boot.
  If disabled, ``yrk-core.py`` will not be run.  This is often useful for testing but user needs to
  remember to keep check on battery and temperature.
* Switch 1 enables the ROS service using ROS launch.  If the switch is disabled after ROS has been
  launched the process will be killed, allowing a relaunch.
* Switch 2 enables the web service.  This enables a **Flask** webserver running a **Dash** site with **DAQ**
  components and this manual.  By default at ``localhost:8080``.
* Switch 3 enables the demo program.  [To do...]


Console
+++++++

A **curses** based shell console program has been written that provides a useful quick test for
hardware and also a useful example of how to do many low-level API calls.  The console should be run
without any other code (include ``yrk-core.py``) running.  It can be run as follows::

   cd ~/yrk/examples
   python console.py


.. figure:: /images/console.png
     :width: 550px
     :height: 347px
     :alt: Screen shot of console.py python program

     Screen shot of ``console.py``

If you are using **ssh** you may find the line drawing characters are not represented properly.  If using
Windows, *kitty* (a fork of *putty*) has a ``Allow ACS line drawing in UTF`` setting which allows the
correct rendering.

The cursor keys on the keyboard can be used to move between the different motor, LED and servo options
(*the console sets the PWM driver up for 1.5mS analogue servos*).  As the console can enable and motors
and servos it should be used with caution.
