# ROS2_Audio_Tools
SDL2 based audio tools for ROS2 (written in C++).

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