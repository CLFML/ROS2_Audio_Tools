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