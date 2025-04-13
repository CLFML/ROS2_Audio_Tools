# EdgeVox Project Documentation

# Project Structure
```
ROS2_Audio_Tools/
├── conda/
│   ├── lin/
│   │   ├── recipe/
│   │   │   ├── activate.sh
│   │   │   ├── build_ament_cmake.sh
│   │   │   ├── build_ament_python.sh
│   │   │   ├── build_catkin.sh
│   │   │   ├── deactivate.sh
│   │   │   ├── recipe.yaml
│   │   ├── pixi.toml
│   ├── win/
│   │   ├── recipe/
│   │   │   ├── activate.bat
│   │   │   ├── bld_ament_cmake.bat
│   │   │   ├── bld_ament_python.bat
│   │   │   ├── bld_catkin.bat
│   │   │   ├── bld_catkin_merge.bat
│   │   │   ├── bld_colcon_merge.bat
│   │   │   ├── deactivate.bat
│   │   │   ├── recipe.yaml
│   │   │   ├── rosdistro_snapshot.yaml
│   │   ├── pixi.toml
├── docs/
│   ├── about/
│   │   ├── implementation.md
│   ├── assets/
│   │   ├── stylesheets/
│   │   │   ├── logo.css
│   │   ├── favicon.png
│   │   ├── logo.png
│   ├── contributing/
│   │   ├── license.md
│   │   ├── rules.md
│   ├── usage/
│   │   ├── overview.md
│   │   ├── ros2_pixi_build_linux_windows.md
│   │   ├── usage_with_native_linux.md
│   ├── index.md
├── msg/
│   ├── AudioData.msg
│   ├── AudioDataStamped.msg
│   ├── AudioInfo.msg
├── src/
│   ├── audio_capture_node.cpp
│   ├── audio_capture_node.hpp
│   ├── audio_playback_node.cpp
│   ├── audio_playback_node.hpp
├── CMakeLists.txt
├── LICENSE
├── README.md
├── colcon.pkg
├── mkdocs.yml
├── package.xml
├── pixi.toml
```

# src/audio_capture_node.cpp
```cpp
/**
 * @file audio_capture_node.cpp
 * @brief Audio Capture Node for ROS 2 using SDL2
 *
 * Copyright 2025 <Hoog-V / CLFML>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "audio_capture_node.hpp"
#include <stdio.h>

AudioCaptureNode::AudioCaptureNode()
    : Node("audio_capture_node"), _desired_rate(-1.0) {

  // Declare and get parameters
  this->declare_parameter<std::string>("sample_format", "S16LE");
  this->get_parameter("sample_format", _sample_format);

  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("sample_rate", 16000);
  this->get_parameter("channels", _channels);
  this->get_parameter("sample_rate", _sample_rate);

  if (_channels < 1 || _channels > 8) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Invalid channel count: " << _channels);
    throw std::runtime_error("Unsupported media type.");
  }

  this->declare_parameter<int>("device", -1);
  this->get_parameter("device", _device);

  this->declare_parameter<int>("chunk_size", 1024);
  int chunk_size;
  this->get_parameter("chunk_size", chunk_size);

  if (chunk_size < 512 || chunk_size > 4096) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Invalid chunk size: " << chunk_size);
    throw std::runtime_error("Unsupported chunk size!");
  }

  if (!ConfigureSDLStream(_device, _sample_format, chunk_size, _channels)) {
    throw std::runtime_error("Failed to configure SDL audio capture.");
  }

  // Publishers
  _pub = this->create_publisher<audio_tools::msg::AudioData>("audio", 10);
  auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  _pub_info = this->create_publisher<audio_tools::msg::AudioInfo>("audio_info",
                                                                  info_qos);

  _pub_stamped = this->create_publisher<audio_tools::msg::AudioDataStamped>(
      "audio_stamped", 10);

  _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5),
                                     [this] { publishInfo(); });
  publishInfo();
}

void AudioCaptureNode::callback(uint8_t *stream, int len) {
  audio_tools::msg::AudioData msg;
  audio_tools::msg::AudioDataStamped stamped_msg;

  // Generate frame ID
  stamped_msg.header.frame_id = std::to_string(_audio_pos++);
  // Compute timestamp based on buffer size
  size_t buffer_size_total_nsec =
      (static_cast<size_t>(len) * 1e9) / (_sample_rate * _nbytes);

  _nano_sec += buffer_size_total_nsec;
  while (_nano_sec >= RCL_S_TO_NS(1)) {
    _sec++;
    _nano_sec -= RCL_S_TO_NS(1);
  }

  stamped_msg.header.stamp.sec = _sec;
  stamped_msg.header.stamp.nanosec = _nano_sec;

  // Copy audio data
  msg.data.assign(stream, stream + len);

  stamped_msg.audio = msg;
  stamped_msg.info = _info_msg;
  // Publish messages
  _pub->publish(msg);
  _pub_stamped->publish(stamped_msg);
}

void AudioCaptureNode::publishInfo() {
  _info_msg.channels = _channels;
  _info_msg.sample_rate = _sample_rate;
  _info_msg.sample_format = _sample_format;

  _pub_info->publish(_info_msg);
}

AudioCaptureNode::~AudioCaptureNode() {
  SDL_PauseAudioDevice(_dev_id_in, 1);
  SDL_CloseAudioDevice(_dev_id_in);
}

bool AudioCaptureNode::ConfigureSDLStream(int device, const std::string &format,
                                          int chunk_size, int channels) {
  SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);

  if (SDL_Init(SDL_INIT_AUDIO) < 0) {
    RCLCPP_ERROR(this->get_logger(), "SDL initialization failed: %s",
                 SDL_GetError());
    return false;
  }

  SDL_SetHintWithPriority(SDL_HINT_AUDIO_RESAMPLING_MODE, "medium",
                          SDL_HINT_OVERRIDE);

  int nDevices = SDL_GetNumAudioDevices(SDL_TRUE);
  RCLCPP_INFO(this->get_logger(), "%s: Found %d capture devices:\n", __func__,
              nDevices);

  for (int i = 0; i < nDevices; i++) {
    RCLCPP_INFO(this->get_logger(), "  - Capture device #%d: '%s'\n", i,
                SDL_GetAudioDeviceName(i, SDL_TRUE));
  }

  SDL_zero(_capture_spec_requested);
  SDL_zero(_capture_spec_obtained);

  auto format_it = _format_map.find(format);
  if (format_it == _format_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Unknown audio format: %s",
                 format.c_str());
    return false;
  }
  _nbytes = format_it->second.second;

  _capture_spec_requested.freq = _sample_rate;
  _capture_spec_requested.format = format_it->second.first;
  _capture_spec_requested.channels = channels;
  _capture_spec_requested.samples = chunk_size;
  _capture_spec_requested.callback = [](void *userdata, uint8_t *stream,
                                        int len) {
    static_cast<AudioCaptureNode *>(userdata)->callback(stream, len);
  };
  _capture_spec_requested.userdata = this;

  if (device >= 0 && device < nDevices) {
    RCLCPP_INFO(this->get_logger(),
                "%s: Attempting to open capture device #%d: '%s'...\n",
                __func__, device, SDL_GetAudioDeviceName(device, SDL_TRUE));

    _dev_id_in = SDL_OpenAudioDevice(SDL_GetAudioDeviceName(device, SDL_TRUE),
                                     SDL_TRUE, &_capture_spec_requested,
                                     &_capture_spec_obtained, 0);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s: Attempting to open default capture device...\n", __func__);
    _dev_id_in =
        SDL_OpenAudioDevice(nullptr, SDL_TRUE, &_capture_spec_requested,
                            &_capture_spec_obtained, 0);
  }

  if (!_dev_id_in) {
    RCLCPP_INFO(this->get_logger(), "%s: Failed to open audio device: %s\n",
                __func__, SDL_GetError());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "%s: Capture device opened (SDL Id = %d):\n",
              __func__, _dev_id_in);
  RCLCPP_INFO(this->get_logger(), "  - Sample rate: %d\n",
              _capture_spec_obtained.freq);
  RCLCPP_INFO(this->get_logger(), "  - Format: %d (Requested: %d)\n",
              _capture_spec_obtained.format, _capture_spec_requested.format);
  RCLCPP_INFO(this->get_logger(), "  - Channels: %d (Requested: %d)\n",
              _capture_spec_obtained.channels,
              _capture_spec_requested.channels);
  RCLCPP_INFO(this->get_logger(), "  - Samples per frame: %d\n",
              _capture_spec_obtained.samples);

  _sample_rate = _capture_spec_obtained.freq;
  SDL_PauseAudioDevice(_dev_id_in, 0);
  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
```


# src/audio_capture_node.hpp
```cpp
/**
 * @file audio_capture_node.hpp
 * @brief Audio Capture Node for ROS 2 using SDL2
 *
 * Copyright 2025 <Hoog-V / CLFML>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef AUDIO_CAPTURE_NODE_HPP
#define AUDIO_CAPTURE_NODE_HPP

#include <SDL.h>
#include <SDL_audio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_tools/msg/audio_data.hpp"
#include "audio_tools/msg/audio_data_stamped.hpp"
#include "audio_tools/msg/audio_info.hpp"

/**
 * @class AudioCaptureNode
 * @brief A ROS 2 node for capturing and publishing audio data using SDL2.
 */
class AudioCaptureNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the AudioCaptureNode.
   * Initializes audio capture, publishers, and diagnostics.
   */
  AudioCaptureNode();

  /**
   * @brief Exits the program with the given exit code.
   * @param code The exit code.
   */
  void exitOnMainThread(int code);

  /**
   * @brief SDL2 audio callback function.
   * Processes captured audio and publishes it as a ROS message.
   * @param stream Pointer to the audio stream buffer.
   * @param len Length of the audio buffer in bytes.
   */
  void callback(uint8_t *stream, int len);

  /**
   * @brief Publishes audio information (sample rate, channels, format).
   */
  void publishInfo();

  /**
   * @brief Destructor for AudioCaptureNode.
   * Cleans up SDL audio resources.
   */
  ~AudioCaptureNode();

private:
  /**
   * @brief ROS publisher for raw audio data.
   */
  rclcpp::Publisher<audio_tools::msg::AudioData>::SharedPtr _pub;

  /**
   * @brief ROS publisher for audio metadata (e.g., sample rate, format).
   */
  rclcpp::Publisher<audio_tools::msg::AudioInfo>::SharedPtr _pub_info;

  /**
   * @brief Timer for periodically publishing audio metadata.
   */
  rclcpp::TimerBase::SharedPtr _timer_info;

  /**
   * @brief Audio information metadata
   */
  audio_tools::msg::AudioInfo _info_msg;

  /**
   * @brief SDL audio specifications for requested and obtained formats.
   */
  SDL_AudioSpec _capture_spec_requested;
  SDL_AudioSpec _capture_spec_obtained;

  /**
   * @brief Audio parameters: bitrate, channels, depth, sample rate, and device
   * index.
   */
  int _bitrate, _channels, _sample_rate, _device;

  /**
   * @brief Number of bytes processed.
   */
  size_t _nbytes;

  /**
   * @brief Position counter for audio frames.
   */
  size_t _audio_pos = 0;

  /**
   * @brief Nanosecond and second timestamps for synchronization.
   */
  size_t _nano_sec = 0;
  size_t _sec = 0;

  /**
   * @brief Desired publishing rate for diagnostics.
   */
  double _desired_rate;

  /**
   * @brief Audio sample format (e.g., "S16LE", "F32").
   */
  std::string _sample_format;

  /**
   * @brief SDL audio device ID.
   */
  SDL_AudioDeviceID _dev_id_in = 0;

  /**
   * @brief ROS publisher for timestamped audio data with diagnostics.
   */
  rclcpp::Publisher<audio_tools::msg::AudioDataStamped>::SharedPtr _pub_stamped;

  /**
   * @brief Configures the SDL2 audio capture stream.
   * @param device The SDL2 device index (-1 for default).
   * @param format The requested audio format (e.g., "S16LE").
   * @param chunk_size The buffer size for each audio chunk (e.g., 1024
   * samples).
   * @param channels The number of audio channels (e.g., 1 for mono, 2 for
   * stereo).
   * @return True if configuration is successful, false otherwise.
   */
  bool ConfigureSDLStream(int device, const std::string &format, int chunk_size,
                          int channels);

  /**
   * @brief Maps audio format strings to SDL2 audio formats and corresponding
   * byte sizes.
   */
  const std::unordered_map<std::string, std::pair<SDL_AudioFormat, int>>
      _format_map = {
          {"U8", {AUDIO_U8, 1}}, ///< Unsigned 8-bit (1 byte)
          {"S8", {AUDIO_S8, 1}}, ///< Signed 8-bit (1 byte)
          {"U16LE",
           {AUDIO_U16LSB, 2}}, ///< Unsigned 16-bit little-endian (2 bytes)
          {"U16BE",
           {AUDIO_U16MSB, 2}}, ///< Unsigned 16-bit big-endian (2 bytes)
          {"S16LE",
           {AUDIO_S16LSB, 2}}, ///< Signed 16-bit little-endian (2 bytes)
          {"S16BE", {AUDIO_S16MSB, 2}}, ///< Signed 16-bit big-endian (2 bytes)
          {"S32LE",
           {AUDIO_S32LSB, 4}}, ///< Signed 32-bit little-endian (4 bytes)
          {"S32BE", {AUDIO_S32MSB, 4}}, ///< Signed 32-bit big-endian (4 bytes)
          {"F32LE",
           {AUDIO_F32LSB, 4}}, ///< 32-bit float little-endian (4 bytes)
          {"F32BE", {AUDIO_F32MSB, 4}}, ///< 32-bit float big-endian (4 bytes)
          {"F32", {AUDIO_F32, 4}}       ///< 32-bit float (generic) (4 bytes)
      };
};

#endif /* AUDIO_CAPTURE_NODE_HPP */

```


# src/audio_playback_node.cpp
```cpp
/**
 * @file audio_playback_node.cpp
 * @brief Audio Capture Node for ROS 2 using SDL2
 *
 * Copyright 2025 <Hoog-V / CLFML>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "audio_playback_node.hpp"
#include <stdio.h>

AudioPlaybackNode::AudioPlaybackNode()
    : Node("audio_playback_node"), _audio_device(0), _initialized(false) {
  // Declare and get topic name parameter
  std::string topic_name =
      this->declare_parameter<std::string>("audio_topic", "/audio_playback");
  int device_index =
      this->declare_parameter<int>("device_index", -1); // -1 = default

  _subscription = this->create_subscription<audio_tools::msg::AudioDataStamped>(
      topic_name, 10,
      std::bind(&AudioPlaybackNode::audio_cb, this, std::placeholders::_1));

  if (SDL_Init(SDL_INIT_AUDIO) < 0) {
    RCLCPP_FATAL(get_logger(), "SDL_Init failed: %s", SDL_GetError());
    rclcpp::shutdown();
  }
  int num_devices = SDL_GetNumAudioDevices(0);
  RCLCPP_INFO(get_logger(), "Available audio output devices:");
  for (int i = 0; i < num_devices; ++i) {
    const char *name = SDL_GetAudioDeviceName(i, 0);
    RCLCPP_INFO(get_logger(), "  [%d]: %s%s", i, name,
                (i == device_index ? " (selected)" : ""));
  }

  if (device_index >= 0 && device_index < num_devices) {
    _device_name = SDL_GetAudioDeviceName(device_index, 0);
    RCLCPP_INFO(get_logger(), "Using audio device: %s", _device_name.c_str());
  } else if (device_index == -1) {
    _device_name = ""; // SDL default
    RCLCPP_INFO(get_logger(), "Using default audio device");
  } else {
    RCLCPP_FATAL(get_logger(), "Invalid device_index: %d (available: 0 to %d)",
                 device_index, num_devices - 1);
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "Subscribed to topic: %s", topic_name.c_str());
}

AudioPlaybackNode::~AudioPlaybackNode() {
  if (_audio_device != 0) {
    SDL_CloseAudioDevice(_audio_device);
  }
  SDL_Quit();
}

bool AudioPlaybackNode::init_audio(const audio_tools::msg::AudioInfo &info) {
  SDL_AudioSpec desired_spec;
  SDL_zero(desired_spec);

  desired_spec.freq = static_cast<int>(info.sample_rate);
  desired_spec.channels = info.channels;
  desired_spec.samples = 4096;
  desired_spec.callback = nullptr;

  if (info.sample_format == "S16LE") {
    desired_spec.format = AUDIO_S16LSB;
  } else if (info.sample_format == "S16BE") {
    desired_spec.format = AUDIO_S16MSB;
  } else if (info.sample_format == "U8") {
    desired_spec.format = AUDIO_U8;
  } else if (info.sample_format == "S8") {
    desired_spec.format = AUDIO_S8;
  } else if (info.sample_format == "F32LE") {
    desired_spec.format = AUDIO_F32LSB;
  } else if (info.sample_format == "F32BE") {
    desired_spec.format = AUDIO_F32MSB;
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported format: %s",
                 info.sample_format.c_str());
    return false;
  }

  _audio_device =
      SDL_OpenAudioDevice(_device_name.empty() ? nullptr : _device_name.c_str(),
                          0, // 0 = output
                          &desired_spec, &_obtained_spec, 0);
  if (_audio_device == 0) {
    RCLCPP_ERROR(get_logger(), "SDL_OpenAudioDevice error: %s", SDL_GetError());
    return false;
  }

  SDL_PauseAudioDevice(_audio_device, 0);
  RCLCPP_INFO(get_logger(), "Audio device initialized: %d Hz, %d channels",
              _obtained_spec.freq, _obtained_spec.channels);

  _initialized = true;
  return true;
}

void AudioPlaybackNode::audio_cb(
    const audio_tools::msg::AudioDataStamped::SharedPtr msg) {
  const auto &info = msg->info;
  const auto &audio = msg->audio;

  if (!_initialized) {
    if (!init_audio(info)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize audio output.");
      return;
    }
  }

  if (audio.data.empty())
    return;

  if (SDL_QueueAudio(_audio_device, audio.data.data(), audio.data.size()) < 0) {
    RCLCPP_ERROR(get_logger(), "SDL_QueueAudio error: %s", SDL_GetError());
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioPlaybackNode>());
  rclcpp::shutdown();
  return 0;
}
```


# src/audio_playback_node.hpp
```cpp
/**
 * @file audio_playback_node.hpp
 * @brief Audio Playback Node for ROS 2 using SDL2
 *
 * Copyright 2025 <Hoog-V / CLFML>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef AUDIO_PLAYBACK_NODE_HPP
#define AUDIO_PLAYBACK_NODE_HPP
#include <SDL.h>
#include <SDL_audio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_tools/msg/audio_data.hpp"
#include "audio_tools/msg/audio_data_stamped.hpp"
#include "audio_tools/msg/audio_info.hpp"

/**
 * @class AudioPlaybackNode
 * @brief A ROS 2 node for capturing and publishing audio data using SDL2.
 */
class AudioPlaybackNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the AudioPlaybackNode.
   * Initializes audio capture, publishers, and diagnostics.
   */
  AudioPlaybackNode();

  /**
   * @brief Destructor for AudioPlaybackNode.
   * Cleans up SDL audio resources.
   */
  ~AudioPlaybackNode();

private:
  /**
   * @brief Callback function for received audio_msg on subscribed topic
   * @param msg The audio message which got received
   */
  void audio_cb(const audio_tools::msg::AudioDataStamped::SharedPtr msg);
  /**
   * @brief Function which automatically configures the audio_interface based on
   * the received audio information
   * @param info Custom message which contains the channels, sample_rate and
   * encoding format
   * @return false if failed e.g. invalid format, true if ok
   */
  bool init_audio(const audio_tools::msg::AudioInfo &info);

  /**
   * @brief Subscription object/handle for the subscribed audio topic
   */
  rclcpp::Subscription<audio_tools::msg::AudioDataStamped>::SharedPtr
      _subscription;

  /**
   * @brief SDL audio device handle.
   */
  SDL_AudioDeviceID _audio_device;

  /**
   * @brief SDL audio specifications for obtained format.
   */
  SDL_AudioSpec _obtained_spec;

  /**
   * @brief Full name of audio_device
   */
  std::string _device_name;

  /**
   * @brief Internal state to prevent initializing audio interface twice
   */
  bool _initialized;
};
#endif /* AUDIO_PLAYBACK_NODE */
```


# CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.1...3.14)
project(audio_tools C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AudioData.msg
  msg/AudioDataStamped.msg
  msg/AudioInfo.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(audio_capture_node src/audio_capture_node.cpp)

ament_target_dependencies(audio_capture_node
    rclcpp
    std_msgs
)

target_link_libraries(audio_capture_node "${cpp_typesupport_target}" ${SDL2_LIBRARIES}) 
target_include_directories(audio_capture_node PUBLIC ${SDL2_INCLUDE_DIRS})

if(UNIX AND NOT APPLE)  # Linux only
  target_compile_options(audio_capture_node PRIVATE -Wall)
endif()

install(TARGETS
audio_capture_node
DESTINATION lib/${PROJECT_NAME}) 


add_executable(audio_playback_node src/audio_playback_node.cpp)
target_link_libraries(audio_playback_node "${cpp_typesupport_target}" ${SDL2_LIBRARIES}) 
target_include_directories(audio_playback_node PUBLIC ${SDL2_INCLUDE_DIRS})

if(UNIX AND NOT APPLE)
  target_compile_options(audio_playback_node PRIVATE -Wall)
endif()

ament_target_dependencies(audio_playback_node
    rclcpp
    std_msgs
)


install(TARGETS
audio_playback_node
DESTINATION lib/${PROJECT_NAME}) 
ament_package()
```


# mkdocs.yml
```text
site_name: ROS2_Audio_Tools
extra_css:
  - assets/stylesheets/logo.css
theme:
  name: material
  # custom_dir: docs/overrides
  features:
    - announce.dismiss
    - content.action.edit
    - content.action.view
    - content.code.annotate
    - content.code.copy
    # - content.code.select
    # - content.footnote.tooltips
    # - content.tabs.link
    - content.tooltips
    # - header.autohide
    # - navigation.expand
    - navigation.footer
    - navigation.indexes
    # - navigation.instant
    # - navigation.instant.prefetch
    # - navigation.instant.progress
    # - navigation.prune
    - navigation.sections
    - navigation.tabs
    # - navigation.tabs.sticky
    - navigation.top
    - navigation.tracking
    - search.highlight
    - search.share
    - search.suggest
    - toc.follow
    # - toc.integrate
  palette:
      scheme: default
      primary: green
      toggle:
        icon: material/toggle-switch
        name: Switch to dark mode
  font:
    text: Avenir Next Condensed
    code: Avenir Next Condensed
  favicon: assets/favicon.png
  logo: assets/logo.png

markdown_extensions:
  - attr_list
  - md_in_html
  - admonition
  - pymdownx.details
  - pymdownx.superfences

nav:
  - index.md
  - Getting Started:
    - usage/overview.md
    - Build Environment:
      - usage/ros2_pixi_build_linux_windows.md
      - usage/usage_with_native_linux.md
  - Implementation:
    - about/implementation.md
  - Contributing:
    - contributing/rules.md
    - contributing/license.md
```

