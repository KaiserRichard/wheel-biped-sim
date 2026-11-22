#!/usr/bin/env bash

export RUNTIME_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Separate this project from unrelated ROS systems on the lab network.
export ROS_DOMAIN_ID=42

# Match the RealSense container's FastDDS setting.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

echo "RUNTIME_ROOT=$RUNTIME_ROOT"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
