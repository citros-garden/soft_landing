#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/citros_soft_landing/install/setup.bash 

exec "$@"