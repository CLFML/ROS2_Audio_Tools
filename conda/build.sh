#!/bin/bash
set -eux

colcon build --merge-install --install-base=$CONDA_PREFIX
