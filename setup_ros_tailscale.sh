#!/bin/bash

# Set up the network and DDS environment variables for Tailscale
export CYCLONEDDS_URI=file://$PIXI_PROJECT_ROOT/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_NETWORK_INTERFACE=tailscale0
export ROS_IP=$(tailscale ip -4)
export ROS_DOMAIN_ID=0

# Source the ROS workspace
source install/setup.bash

# Execute the command passed as arguments
exec "$@"

echo "ROS2 environment configured for Tailscale communication"
