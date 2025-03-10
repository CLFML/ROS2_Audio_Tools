# ROS2_Audio_Tools
SDL2 based audio tools for ROS2 (written in C++).
[![License: MIT](https://img.shields.io/badge/GitHub-Apache_2-informational)](https://opensource.org/license/apache) [![GitHub release](https://img.shields.io/github/release/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/releases) [![Code Size](https://img.shields.io/github/languages/code-size/CLFML/ROS2_audio_tools.svg?branch=main)](https://github.com/CLFML/ROS2_audio_tools?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/commits/main) [![GitHub issues](https://img.shields.io/github/issues/CLFML/ROS2_audio_tools)](https://github.com/CLFML/ROS2_audio_tools/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/CLFML/ROS2_audio_tools)](https://github.com/CLFML/ROS2_audio_tools/pulls) [![Contributors](https://img.shields.io/github/contributors/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/graphs/contributors) [![C++ Formatter Check](https://github.com/CLFML/ROS2_audio_tools/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/CLFML/ROS2_audio_tools/actions/workflows/cpp-formatter.yml?branch=main)

## Features
This package contains an audio capture node which captures from a local capture device such as a microphone.

In the future this package will be expanded with a Playback and VAD node as well.

## Getting started
To get started you need to get yourself a copy of this library. You can do this by fetching the binary distribution (when using Ubuntu/Debian) or manually building the library.

### Binary distribution
For debian there are binary distributions of this library in the releases section of this repository.

### Manual build
Run these commands:
#### 1 Create a workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
#### 2 Clone the repo
```bash
git clone https://github.com/CLFML/ROS2_Audio_Tools.git
```
#### 3 Install dependencies & build
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Todo
- Audio playback node
- VAD node
- Accoustic noise cancelling?

## License
This work is licensed under the Apache 2.0 license. 