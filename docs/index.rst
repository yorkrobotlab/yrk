.. include global.rst

.. YRK Python documentation master file

York Robotics Kit Python API Documentation
==========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Core Modules
============

The core API python modules are found in the ``yrk`` subfolder.

yrk.adc
-------
.. automodule:: yrk.adc
   :members:

yrk.audio
---------
.. automodule:: yrk.audio
  :members:

yrk.display
-----------
.. automodule:: yrk.display
  :members:

yrk.gpio
--------
.. automodule:: yrk.gpio
  :members:

yrk.led
-------
.. automodule:: yrk.led
  :members:

yrk.motors
----------
.. automodule:: yrk.motors
  :members:

yrk.power
---------
.. automodule:: yrk.power
  :members:

yrk.pwm
-------
  .. automodule:: yrk.pwm
    :members:

yrk.settings
------------
.. automodule:: yrk.settings
  :members:

yrk.switch
----------
.. automodule:: yrk.switch
  :members:

yrk.utils
---------
.. automodule:: yrk.utils
  :members:

yrk.yrk-core
------------
.. automodule:: yrk.yrk-core
  :members:

Example Programs
================

Some simple example code programs and utilities are found in the ``examples`` subfolder.

examples.console
----------------
.. automodule:: examples.console
  :members:

examples.potmotor
-----------------
.. automodule:: examples.potmotor
  :members:

examples.potservo
-----------------
.. automodule:: examples.potservo
  :members:

examples.stop
-------------
.. automodule:: examples.stop
  :members:

YRK ROS API
===========

The ROS script, message and service files are found in the catkin workspace subfolder ``catwin_ws/src/yrk_ros``.

This can be quickly accessed from a terminal using the following ROS command::

  roscd yrk_ros


yrk_ros.adc_publisher
---------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.adc_publisher
  :members:

yrk_ros.button_publisher
------------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.button_publisher
  :members:

yrk_ros.display_server
----------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.display_server
  :members:


yrk_ros.led_server
------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.led_server
  :members:


yrk_ros.motor_server
--------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.motor_server
  :members:

yrk_ros.power_monitor
---------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.power_monitor
  :members:

yrk_ros.switched_output_server
------------------------------
.. automodule:: catkin_ws.src.yrk_ros.scripts.switched_output_server
  :members:


Index
=====

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
