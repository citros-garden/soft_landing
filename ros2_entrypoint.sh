#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/soft_landing/install/setup.bash 

exec "$@"