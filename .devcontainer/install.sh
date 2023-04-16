#!/bin/bash
WORKSPACE_DIR="/workspaces/soft_landing/"

cd $WORKSPACE_DIR

git submodule update --init --recursive

pip install -e /workspaces/soft_landing/src/lopt_dynamics/resource/lopt/

echo "Done installing, ready to develop!"