# Audio Tools Node implementations

Here some brief details as how the functionality is implemented.


## `audio_capture_node`

**Audio Capture Node using SDL2 for ROS 2**

Captures raw audio from an input device and publishes it via ROS topics. Useful for audio processing pipelines, speech recognition, or diagnostics.

---

### ✅ Topics

| Topic             | Type                                     | Description                       |
|------------------|------------------------------------------|-----------------------------------|
| `/audio`         | `audio_tools/msg/AudioData`              | Raw audio byte stream             |
| `/audio_stamped` | `audio_tools/msg/AudioDataStamped`       | Audio with timestamp + info       |
| `/audio_info`    | `audio_tools/msg/AudioInfo`              | Metadata (sample rate, format, etc.) |

---

### ⚙️ Parameters

| Name           | Type     | Default | Description                                 |
|----------------|----------|---------|---------------------------------------------|
| `sample_format`| `string` | `S16LE` | Audio format (`S16LE`, `F32LE`, etc.)       |
| `channels`     | `int`    | `1`     | Number of channels (1–8)                    |
| `sample_rate`  | `int`    | `16000` | Sampling rate in Hz                         |
| `device`       | `int`    | `-1`    | Device index (-1 = default input device)    |
| `chunk_size`   | `int`    | `1024`  | Buffer size per capture call (512–4096)     |

---

### 📦 Message Notes

- `AudioData`: raw `uint8[] data`
- `AudioDataStamped`:
  - `std_msgs/Header header` with `stamp` and `frame_id`
  - `AudioData audio`
  - `AudioInfo info`
- `AudioInfo` includes:
  - `sample_rate`
  - `sample_format`
  - `channels`

---

### 🧩 Implementation Notes

- Uses **SDL2** for low-latency cross-platform audio capture.
- Device info is printed on startup.
- Buffer timestamping is calculated based on sample rate and byte depth.
- Publishes metadata (`/audio_info`) every 5 seconds.
- All audio capture runs on a callback triggered by SDL2.

---

### 🏁 Launch Example

```bash
ros2 run audio_tools audio_capture_node \
  --ros-args \
  -p sample_rate:=16000 \
  -p channels:=1 \
  -p device:=-1
```

---

### 🧪 Dependencies

- `rclcpp`
- `SDL2`
- Custom messages in `audio_tools`



## `audio_playback_node`

**Audio Playback Node using SDL2 for ROS 2**

Subscribes to audio messages and plays them in real-time through an output device using SDL2.

---

### ✅ Topics

| Topic             | Type                                     | Description                    |
|------------------|------------------------------------------|--------------------------------|
| `/audio_playback` (default) | `audio_tools/msg/AudioDataStamped` | Audio stream with metadata     |

---

### ⚙️ Parameters

| Name           | Type     | Default         | Description                                  |
|----------------|----------|-----------------|----------------------------------------------|
| `audio_topic`  | `string` | `/audio_playback` | Topic to subscribe to for audio data        |
| `device_index` | `int`    | `-1`             | Index of output device (-1 = default device) |

---

### 📦 Expected Message Format

- `AudioDataStamped`:
  - `AudioData audio`: raw audio byte stream
  - `AudioInfo info`: playback metadata:
    - `sample_rate`
    - `sample_format`
    - `channels`

---

### 🧩 Implementation Notes

- Uses **SDL2** to output audio through a selected or default device.
- Supports multiple formats: `S16LE`, `S16BE`, `U8`, `S8`, `F32LE`, `F32BE`
- Device info printed on startup.
- Initializes playback only on receiving the first message (lazy init).
- Queues audio chunks using `SDL_QueueAudio()`.

---

### 🏁 Launch Example

```bash
ros2 run audio_tools audio_playback_node \
  --ros-args \
  -p audio_topic:=/audio_stamped \
  -p device_index:=-1
```

---

### 🧪 Dependencies

- `rclcpp`
- `SDL2`
- Custom messages in `audio_tools`

