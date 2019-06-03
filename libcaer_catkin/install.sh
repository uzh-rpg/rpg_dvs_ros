#!/bin/bash
echo "Copying udev rule (needs root privileges)."
sudo cp 65-inivation.rules /etc/udev/rules.d/

echo "Reloading udev rules."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done!"
