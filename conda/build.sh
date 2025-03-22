#!/bin/bash
set -eux

# Package your pre-built ROS 2 node (from .pixi environment)
mkdir -p $PREFIX/include
mkdir -p $PREFIX/lib
mkdir -p $PREFIX/share

cp -r .pixi/envs/default/audio_tools/include/* $PREFIX/include/
cp -r .pixi/envs/default/audio_tools/lib/* $PREFIX/lib/
cp -r .pixi/envs/default/audio_tools/share/* $PREFIX/share/
