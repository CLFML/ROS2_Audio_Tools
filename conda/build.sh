#!/bin/bash
set -eux

colcon build --merge-install --install-base=$CONDA_PREFIX --cmake-args -DCMAKE_BUILD_TYPE=Release"
