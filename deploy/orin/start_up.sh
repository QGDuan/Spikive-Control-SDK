#!/bin/bash


sleep 3


# Step 1: Source your custom ROS workspace
echo "Step 1: Sourcing your custom ROS workspace..."
source ~/ego_ws/devel/setup.bash
if [ $? -eq 0 ]; then
echo "Custom ROS workspace sourced successfully."
else
echo "Failed to source custom ROS workspace. Exiting."
exit 1
fi


# Launch the first ROS launch file

sleep 5
# Launch the fifth ROS launch file
gnome-terminal --tab  -- bash -c "source /home/colman/driver_ws/devel/setup.bash && roslaunch visual_pointcloud visual_pointcloud.launch; exec bash"


sleep 2
# Launch the fifth ROS launch file
gnome-terminal --tab  -- bash -c "source /home/colman/driver_ws/devel/setup.bash && roslaunch usb_gimbal_camera usb_camera.launch; exec bash"

sleep 2
# Launch the fifth ROS launch file
gnome-terminal --tab  -- bash -c "roslaunch foxglove_bridge foxglove_bridge.launch _drone_id:=2; exec bash"

# Launch all Spikive Control SDK modules.
gnome-terminal --tab --title="Spikive Control SDK" -- bash -c 'source /opt/ros/noetic/setup.bash && source /home/colman/driver_ws/devel/setup.bash && exec roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2 enable_manager:=true manager_config:=$(rospack find spikive_control_sdk)/config/manager/launch_config_airy.yaml'
