#!/bin/bash

echo "=========================="
echo "Removing App visual_side_radar"


LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable visual


# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c visual.service` = "x1" ]; then
    echo "Uninstalling visual.service"

    source /home/$LIBPANDA_USER/catkin_ws/devel/setup.bash
    rosrun robot_upstart uninstall visual
fi

systemctl daemon-reload # if needed
