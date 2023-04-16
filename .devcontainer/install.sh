#!/bin/bash
WORKSPACE_DIR="/workspaces/soft_landing/"

cd $WORKSPACE_DIR

git submodule update --init --recursive

python3 -m pip install -e /home/lulav/dev/citros-garden/soft_landing/src/lopt_dynamics/resource/lopt/

echo "Done installing, ready to develop!"