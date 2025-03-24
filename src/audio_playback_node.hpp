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