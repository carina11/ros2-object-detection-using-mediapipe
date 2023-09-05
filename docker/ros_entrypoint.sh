#!/bin/bash
set -e  # This used to cause Docker to exit when auto-filling or running into error

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "$ROBOT_WS/install/setup.bash"

exec "$@"
