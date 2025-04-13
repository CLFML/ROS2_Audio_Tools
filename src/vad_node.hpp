/**
 * @file vad_node.hpp
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
#ifndef VAD_NODE_HPP
#define VAD_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "audio_tools/msg/audio_data.hpp"
#include "audio_tools/msg/audio_data_stamped.hpp"
#include "audio_tools/msg/audio_info.hpp"
#include "audio_tools/msg/voice_activity.hpp"

/**
 * @class VADNode
 * @brief A ROS 2 node for Voice Activity Detection (VAD) using audio streams.
 */
class VADNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the VADNode.
   * Initializes subscribers, publishers, and parameters.
   */
  VADNode();

  /**
   * @brief Destructor for VADNode.
   */
  ~VADNode() = default;

private:
  /**
   * @brief Callback for audio data messages.
   * @param msg The audio message received from the audio capture node.
   */
  void audioCallback(const audio_tools::msg::AudioDataStamped::SharedPtr msg);

  /**
   * @brief Processes audio data to detect voice activity.
   * @param audio_data The raw audio data samples.
   * @param sample_format The format of the audio samples.
   * @return True if voice activity is detected, false otherwise.
   */
  bool detectVoiceActivity(const std::vector<uint8_t> &audio_data, 
                           const std::string &sample_format);
  
  /**
   * @brief Converts audio data to float samples for processing.
   * @param audio_data The raw audio data.
   * @param sample_format The format of the audio samples.
   * @param out_samples Output vector of float samples.
   * @return True if conversion was successful, false otherwise.
   */
  bool convertToFloatSamples(const std::vector<uint8_t> &audio_data,
                             const std::string &sample_format,
                             std::vector<float> &out_samples);

  /**
   * @brief Calculates the energy level of the audio samples.
   * @param samples The audio samples in float format.
   * @return The calculated energy level.
   */
  float calculateEnergyLevel(const std::vector<float> &samples);

  /**
   * @brief Publishes the current voice activity state.
   * @param active Whether voice activity is detected.
   * @param energy_level The current audio energy level.
   * @param timestamp The timestamp for the message.
   */
  void publishVoiceActivity(bool active, float energy_level, 
                            const rclcpp::Time &timestamp);

  /**
   * @brief ROS publisher for voice activity detection results.
   */
  rclcpp::Publisher<audio_tools::msg::VoiceActivity>::SharedPtr _vad_pub;

  /**
   * @brief ROS subscriber for audio data.
   */
  rclcpp::Subscription<audio_tools::msg::AudioDataStamped>::SharedPtr _audio_sub;

  /**
   * @brief Parameters for VAD.
   */
  float _energy_threshold;  // Threshold for energy-based detection
  float _hold_time;         // Time in seconds to hold detection state
  int _min_samples;         // Minimum number of samples for detection

  /**
   * @brief State variables for VAD.
   */
  bool _current_vad_state;  // Current voice activity state
  rclcpp::Time _last_voice_time;  // Time when voice was last detected
  
  /**
   * @brief Conversion map for audio formats to byte sizes.
   */
  const std::unordered_map<std::string, int> _format_byte_size = {
      {"U8", 1},   {"S8", 1},     {"U16LE", 2}, {"U16BE", 2},
      {"S16LE", 2}, {"S16BE", 2}, {"S32LE", 4}, {"S32BE", 4},
      {"F32LE", 4}, {"F32BE", 4}, {"F32", 4}
  };
};

#endif /* VAD_NODE_HPP */