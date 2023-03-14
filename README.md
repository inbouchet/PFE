# PFE
Projet de fin d'études avec Laëtitia Cabrol (Master 2 ASPIC)

# Install and use Structure Core Camera (PX4 Vision Kit) under Linux
If you plan to use the camera for development with ROS 1, you have to know that the package is
only available under ROS Melodic and ROS Kinectic!

I Installation

First, you will need to download the Cross-Platform SDK Structure from the site:
https://structure.io/developers.

Afterward, thanks to the documentation available in
“StructureSDK-CrossPlatform-0.9/Documentation/linux.html” you can first install the driver and then
update it.

If the message shows that the updater is obsolete, be aware that the link provided is obsolete, the
correct link is:
https://support.structure.io/article/399-can-i-upgrade-my-structure-cores-obsolete-firmware.

You will have to restart the system after each update.

You can after following the instructions use the applications in
“StructureSDK-CrossPlatform-0.9/Apps”.

II ROS

For this part, you will need to go to the “StructureSDK-CrossPlatform-0.9/ROS” folder
The documentation for the installation is located: “Documentation/html/ros_install.html”.

For my part, I didn't use catkin_make but catkin build and couldn't get the example using the PCL
library, if that's a problem as you install the package without the BUILD_EXAMPLES option.

The specifics of the camera and its parameters can be found:
“Documentation/html/ros_main_features.html”.

To know : The camera despite the topic named RGB the camera is in mono and does not display
the colors, I did not use it, but you can test the following git which would allow you to obtain the
colors: https://github.com/chadrockey/structure_core_ros.

To use ROS with the camera, you will also need to read “Documentation/html/ros_start.html”.
