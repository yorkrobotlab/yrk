#!/bin/bash

#Check to see if HOSTNAME is correctly set [based on MAC address] or if it is default [raspberry-pi]
  # Find MAC of eth0, or if not exist wlan0
  if [ -e /sys/class/net/eth0 ]; then
      MAC=$(cat /sys/class/net/eth0/address)
  elif [ -e /sys/class/net/enx* ]; then
      MAC=$(cat /sys/class/net/enx*/address)
  else
      MAC=$(cat /sys/class/net/wlan0/address)
  fi
  #echo MAC Address:$MAC
  TRIMMED_MAC="${MAC:9}"
  STRIPPED_MAC="${TRIMMED_MAC//:}"
  TARGET_HOSTNAME="rpi-${STRIPPED_MAC}"
  #echo Hostname: $TARGET_HOSTNAME

if [ "$HOSTNAME" != "$TARGET_HOSTNAME" ]; then
  echo
  echo The HOSTNAME is currently set to $HOSTNAME but the MAC address is $MAC
  echo The expected hostname is $TARGET_HOSTNAME
  echo
  echo Run \"sudo ./fixhostname.sh\" to update the hostname variable
  echo
fi

#Check to see if this script has already been run by looking for existence of /mnt/ramdisk/selftest.log
if [ ! -f /mnt/ramdisk/selftest.log ]; then
    #Kill all instances of python
    if pgrep -x "python" > /dev/null
    then
      killall python
    fi
    echo
    echo " __     __        _      _____       _           _   _            _  ___ _   ";
    echo " \ \   / /       | |    |  __ \     | |         | | (_)          | |/ (_) |  ";
    echo "  \ \_/ /__  _ __| | __ | |__) |___ | |__   ___ | |_ _  ___ ___  | ' / _| |_ ";
    echo "   \   / _ \| '__| |/ / |  _  // _ \| '_ \ / _ \| __| |/ __/ __| |  < | | __|";
    echo "    | | (_) | |  |   <  | | \ \ (_) | |_) | (_) | |_| | (__\__ \ | . \| | |_ ";
    echo "    |_|\___/|_|  |_|\_\ |_|  \_\___/|_.__/ \___/ \__|_|\___|___/ |_|\_\_|\__|";
    echo "                                                                             ";
    echo "                                                                             ";
    echo "YRL040 - York Robotics Kit - York Robotics Laboratory, Oct 2019"
    echo
    echo "This appears to be the first run of bootscript.sh so will start self-test routine and begin services"
    echo
    python examples/selftest.py
    selftest=$?
    if [[ $selftest == 0 ]]; then
      echo "Starting python services"
      echo
    else echo;echo "Not starting python services.  To autorun services on boot enable";echo "DIP switch 1";echo
    fi
    #python camera.py &
    #python index.py &
    #python core.py &
else
  echo "Not starting python services as files are present on /mnt/ramdisk/"
  echo
  echo "To erase ramdisk and restart services try:"
  echo
  echo ". rerun"
  echo
fi
#python core.py

