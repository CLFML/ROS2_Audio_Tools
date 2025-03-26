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