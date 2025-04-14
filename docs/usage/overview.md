# Getting started

# 🚀 Quick Start Examples

> See [node documentation](../about/implementation.md) above for parameters & message formats.

---

### 🔊 1. Start Audio Capture Node

Capture microphone input and publish to `/audio_stamped`.

```bash
ros2 run audio_tools audio_capture_node
```

Custom sample rate & stereo:

```bash
ros2 run audio_tools audio_capture_node \
  --ros-args -p sample_rate:=44100 -p channels:=2
```

---

### 🎧 2. Start Playback Node (Default Device)

Play back from `/audio_stamped` using system default output.

```bash
ros2 run audio_tools audio_playback_node \
  --ros-args -p audio_topic:=/audio_stamped
```

---

### 🎛️ 3. Use a Specific Output Device

List devices first (done automatically on launch), then select one:

```bash
ros2 run audio_tools audio_playback_node \
  --ros-args -p device_index:=1
```

---

### 🔄 4. Full Loopback Test

Run both nodes to capture & play audio:

```bash
# Terminal 1
ros2 run audio_tools audio_capture_node

# Terminal 2
ros2 run audio_tools audio_playback_node \
  --ros-args -p audio_topic:=/audio_stamped
```

### 🗣️ 5. Voice Activity Detection

Capture microphone input and publish to `/audio_stamped`. Simple energy-based Voice Activity Detection (VAD) node on `/audio_stamped` and publishes detection results to `/voice_activity`. Note that VAD parameters are set in launch file.

```bash
# Terminal 1
ros2 launch audio_tools audio_vad.launch.py

# Terminal 2
ros2 topic echo /voice_activity
```
