# ROS2_Audio_Tools
SDL2 based audio tools for ROS2 (written in C++).
[![License: MIT](https://img.shields.io/badge/GitHub-Apache_2-informational)](https://opensource.org/license/apache) [![GitHub release](https://img.shields.io/github/release/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/releases) [![Code Size](https://img.shields.io/github/languages/code-size/CLFML/ROS2_audio_tools.svg?branch=main)](https://github.com/CLFML/ROS2_audio_tools?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/commits/main) [![GitHub issues](https://img.shields.io/github/issues/CLFML/ROS2_audio_tools)](https://github.com/CLFML/ROS2_audio_tools/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/CLFML/ROS2_audio_tools)](https://github.com/CLFML/ROS2_audio_tools/pulls) [![Contributors](https://img.shields.io/github/contributors/CLFML/ROS2_audio_tools.svg)](https://github.com/CLFML/ROS2_audio_tools/graphs/contributors) [![C++ Formatter Check](https://github.com/CLFML/ROS2_audio_tools/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/CLFML/ROS2_audio_tools/actions/workflows/cpp-formatter.yml?branch=main)

## Features
This package contains an audio capture node which captures from a local capture device such as a microphone. This package also contains an audio playback node, which plays audio samples from a subscribed ROS2 topic through your speakers. 

In the future this package will be expanded with a VAD node as well.

## Getting started
To get started you need to get yourself a copy of this library. You can do this by fetching the binary distribution (when using Ubuntu/Debian) or manually building the library.
See the [Getting started guide](https://clfml.github.io/ROS2_Audio_Tools/usage/overview/)

### Binary distribution
For debian there are binary distributions of this library in the releases section of this repository.

### Pixi/Conda packages
These can be found in our own conda channel:
```
"https://clfml.github.io/conda_ros2_jazzy_channel/"
```
There are packages available for Windows & Linux.
For more details see the [wiki](https://clfml.github.io/ROS2_Audio_Tools/ros2_pixi_build_linux_windows/)

## Todo
- VAD node
- Accoustic noise cancelling?

## License
This work is licensed under the Apache 2.0 license. 