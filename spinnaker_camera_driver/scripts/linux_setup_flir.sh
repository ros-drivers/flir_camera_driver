#!/bin/bash -e

echo ""
echo -e "\e[32mStarting spinnaker_camera_driver setup script\e[0m"
echo ""

sudo addgroup flirimaging
sudo usermod -a -G flirimaging ${USER}

val=$(< /sys/module/usbcore/parameters/usbfs_memory_mb)
if [ "$val" -lt "1000" ]; then
  if [ -e /etc/default/grub ]; then
    if [ $(grep -c "usbcore.usbfs_memory_mb=1000" /etc/default/grub) -eq 0 ]; then
      sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=1000/' /etc/default/grub
      echo "Increased the usbfs memory limits in the default grub configuration. Updating grub"
      sudo update-grub
    else
      echo -e "\e[33mWarn: usbfs memory limit is already set in /etc/default/grub. No changes made, try rebooting the computer\e[0m"
    fi

  else
    echo -e "\e[33mWarn: /etc/default/grub configuration file not found, no changes made. usbfs_memory_mb must be set manually. See docs/linux_setup_flir.md for instructions\e[0m"
    exit 0
  fi
else
  echo "usbfs_memory_mb is already set to $val, no changes necessary."
fi

echo -e "\e[32mDone: spinnaker_camera_driver setup script\e[0m"
echo "Please reboot the device to ensure changes have taken full effect"
echo ""