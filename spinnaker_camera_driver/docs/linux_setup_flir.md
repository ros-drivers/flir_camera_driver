# Manual setup steps

Only use these instructions if you did not install the Spinnaker SDK on
your machine.

## Add the "flirimaging" group and make yourself a member of it
```bash
sudo addgroup flirimaging
sudo usermod -a -G flirimaging ${USER}
```

## Bump the usbfs memory limits
The following was taken from [here](https://www.flir.com/support-center/iis/machine-vision/application-note/using-linux-with-usb-3.1/).
Edit the file ``/etc/default/grub`` and change the line default to:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"
```
Then
```
sudo update-grub
```
If your system does not have ``/etc/default/grub``, create the file ``/etc/rc.local``, and change its permissions to 'executable'. Then write the following text to it:
```
#!/bin/sh -e
sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

exit 0
```

## Setup udev rules
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", GROUP="flirimaging"' | sudo tee -a /etc/udev/rules.d/40-flir-spinnaker.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1724", GROUP="flirimaging"' | sudo tee -a /etc/udev/rules.d/40-flir-spinnaker.rules
sudo service udev restart
sudo udevadm trigger
```

## Logout and log back in (or better, reboot)

``
sudo reboot
``
