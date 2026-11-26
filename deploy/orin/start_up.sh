#!/bin/bash

sleep 3
if ! source /opt/ros/noetic/setup.bash ||
   ! source "$HOME/ego_ws/devel/setup.bash" --extend ||
   ! source "$HOME/driver_ws/devel/setup.bash" --extend; then
    echo 'Spikive startup: failed to load the ROS workspaces.' >&2
    exit 1
fi

launch_tab() {
    local title=$1
    shift
    gnome-terminal --tab --title="$title" -- bash -c '"$@"; exec bash' bash "$@"
}

sdk_dir=$(rospack find spikive_control_sdk) || exit 1
manager_config="$sdk_dir/config/manager/launch_config_airy.yaml"
launch_tab "Spikive Control SDK" roslaunch spikive_control_sdk spikive_control_sdk.launch \
    drone_id:=2 enable_manager:=true "manager_config:=$manager_config"

sleep 5
launch_tab "Foxglove Bridge" roslaunch foxglove_bridge foxglove_bridge.launch _drone_id:=2

camera_device=${SPIKIVE_CAMERA_DEVICE:-/dev/video0}
if [[ ! -c "$camera_device" ]]; then
    printf 'Spikive startup: skipping USB camera because %s is missing.\n' "$camera_device" >&2
elif ! roslaunch --args /gimbal_image_rotate usb_gimbal_camera usb_camera.launch \
        "video_device:=$camera_device" >/dev/null; then
    printf 'Spikive startup: skipping USB camera because gimbal_image_rotate_node is unavailable.\n' >&2
else
    launch_tab "Spikive USB Camera" roslaunch usb_gimbal_camera usb_camera.launch \
        "video_device:=$camera_device"
fi
