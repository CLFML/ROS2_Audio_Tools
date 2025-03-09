/**
 * @file audio_capture_node.hpp
 * @brief Audio Capture Node for ROS 2 using SDL2
 *
 * Copyright 2024 <Hoog-V / CLFML>
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
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
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
   * @brief SDL audio specifications for requested and obtained formats.
   */
  SDL_AudioSpec capture_spec_requested;
  SDL_AudioSpec capture_spec_obtained;

  /**
   * @brief Audio parameters: bitrate, channels, depth, sample rate, and device
   * index.
   */
  int _bitrate, _channels, _depth, _sample_rate, _device;

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
   * @brief ROS diagnostic updater for monitoring audio capture.
   */
  diagnostic_updater::Updater updater_;

  /**
   * @brief ROS publisher for timestamped audio data with diagnostics.
   */
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<
      audio_tools::msg::AudioDataStamped>>
      _diagnosed_pub_stamped;

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
      format_map = {
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
