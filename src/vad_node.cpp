/**
 * @file vad_node.cpp
 * @brief Voice Activity Detection Node for ROS 2
 *
 * Copyright 2025 <ducroq / CLFML>
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
#include "vad_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

VADNode::VADNode() : Node("vad_node"), _current_vad_state(false) {
  // Declare and get parameters
  this->declare_parameter<float>("energy_threshold", 0.01);
  this->declare_parameter<float>("hold_time", 0.5);
  this->declare_parameter<int>("min_samples", 160);
  this->declare_parameter<std::string>("voice_activity_topic",
                                       "voice_activity");
  this->declare_parameter<std::string>("audio_data_topic", "audio_stamped");

  this->get_parameter("energy_threshold", _energy_threshold);
  this->get_parameter("hold_time", _hold_time);
  this->get_parameter("min_samples", _min_samples);
  this->get_parameter("voice_activity_topic", _voice_activity_topic);
  this->get_parameter("audio_data_topic", _audio_data_topic);

  RCLCPP_INFO(this->get_logger(), "VAD initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  - Energy threshold: %.4f",
              _energy_threshold);
  RCLCPP_INFO(this->get_logger(), "  - Hold time: %.2f seconds", _hold_time);
  RCLCPP_INFO(this->get_logger(), "  - Min samples: %d", _min_samples);
  RCLCPP_INFO(this->get_logger(), "  - Voice activity topic: %s",
              _voice_activity_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Audio data topic: %s",
              _audio_data_topic.c_str());

  // Initialize the last voice time to current time
  _last_voice_time = this->now();

  // Create publisher for voice activity detection
  _vad_pub = this->create_publisher<audio_tools::msg::VoiceActivity>(
      _voice_activity_topic, 10);

  // Subscribe to audio data stamped topic
  _audio_sub = this->create_subscription<audio_tools::msg::AudioDataStamped>(
      _audio_data_topic, 10,
      std::bind(&VADNode::audioCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "VAD node is ready");
}

void VADNode::audioCallback(
    const audio_tools::msg::AudioDataStamped::SharedPtr msg) {
  const auto &audio_data = msg->audio.data;
  const auto &audio_info = msg->info;

  // Skip if we have no data
  if (audio_data.empty()) {
    return;
  }

  // Convert audio data to float samples for processing
  std::vector<float> samples;
  if (!convertToFloatSamples(audio_data, audio_info.sample_format, samples)) {
    RCLCPP_WARN(this->get_logger(), "Failed to convert audio format %s",
                audio_info.sample_format.c_str());
    return;
  }

  // Calculate energy level
  float energy_level = calculateEnergyLevel(samples);

  // Detect voice activity
  bool voice_active = (energy_level > _energy_threshold) &&
                      (samples.size() >= static_cast<size_t>(_min_samples));

  // Handle hold time logic
  if (voice_active) {
    _last_voice_time = this->now();

    if (!_current_vad_state) {
      _current_vad_state = true;
      RCLCPP_DEBUG(this->get_logger(), "Voice activity started");
    }
  } else {
    // Check if we're within the hold time
    rclcpp::Duration time_since_voice = this->now() - _last_voice_time;
    if (_current_vad_state && time_since_voice.seconds() > _hold_time) {
      _current_vad_state = false;
      RCLCPP_DEBUG(this->get_logger(), "Voice activity ended");
    }
  }

  // Publish the current VAD state
  publishVoiceActivity(_current_vad_state, energy_level, msg->header.stamp);
}

bool VADNode::detectVoiceActivity(const std::vector<uint8_t> &audio_data,
                                  const std::string &sample_format) {
  // Convert to float samples
  std::vector<float> samples;
  if (!convertToFloatSamples(audio_data, sample_format, samples)) {
    return false;
  }

  // Check if we have enough samples
  if (samples.size() < static_cast<size_t>(_min_samples)) {
    return false;
  }

  // Calculate energy level
  float energy_level = calculateEnergyLevel(samples);

  // Compare with threshold
  return energy_level > _energy_threshold;
}

float VADNode::calculateEnergyLevel(const std::vector<float> &samples) {
  float energy = 0.0f;

  for (const auto &sample : samples) {
    energy += sample * sample;
  }

  // Normalize by number of samples
  if (!samples.empty()) {
    energy /= samples.size();
  }

  return energy;
}

bool VADNode::convertToFloatSamples(const std::vector<uint8_t> &audio_data,
                                    const std::string &sample_format,
                                    std::vector<float> &out_samples) {
  // Find the byte size for this format
  auto it = _format_byte_size.find(sample_format);
  if (it == _format_byte_size.end()) {
    RCLCPP_ERROR(this->get_logger(), "Unknown audio format: %s",
                 sample_format.c_str());
    return false;
  }

  int bytes_per_sample = it->second;
  size_t num_samples = audio_data.size() / bytes_per_sample;
  out_samples.resize(num_samples);

  // Convert based on format
  if (sample_format == "F32" || sample_format == "F32LE") {
    // Direct copy for float format (assuming little endian)
    std::memcpy(out_samples.data(), audio_data.data(), audio_data.size());
  } else if (sample_format == "S16LE") {
    // Convert signed 16-bit LE to float
    const int16_t *int_samples =
        reinterpret_cast<const int16_t *>(audio_data.data());
    for (size_t i = 0; i < num_samples; i++) {
      out_samples[i] = int_samples[i] / 32768.0f;
    }
  } else if (sample_format == "S8") {
    // Convert signed 8-bit to float
    const int8_t *int_samples =
        reinterpret_cast<const int8_t *>(audio_data.data());
    for (size_t i = 0; i < num_samples; i++) {
      out_samples[i] = int_samples[i] / 128.0f;
    }
  } else if (sample_format == "U8") {
    // Convert unsigned 8-bit to float
    for (size_t i = 0; i < num_samples; i++) {
      out_samples[i] = (audio_data[i] - 128) / 128.0f;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Format conversion not implemented for %s",
                 sample_format.c_str());
    throw std::runtime_error("Format conversion not implemented for " +
                             sample_format);
  }

  return true;
}

void VADNode::publishVoiceActivity(bool active, float energy_level,
                                   const rclcpp::Time &timestamp) {
  auto msg = std::make_unique<audio_tools::msg::VoiceActivity>();

  msg->header.stamp = timestamp;
  msg->header.frame_id = "audio_frame";
  msg->active = active;
  msg->energy_level = energy_level;
  msg->threshold = _energy_threshold;
  msg->hold_time = _hold_time;

  _vad_pub->publish(std::move(msg));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VADNode>());
  rclcpp::shutdown();
  return 0;
}