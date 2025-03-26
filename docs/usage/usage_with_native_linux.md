# 🐧 Getting Started with Native ROS

Prefer a traditional system-wide install? Use the prebuilt `.deb` package for **Ubuntu Noble / ROS 2 Jazzy**.

---

## 📦 Install Audio Tools via `.deb`

Install the latest `.deb` package directly from [Releases](https://github.com/CLFML/ROS2_Audio_Tools/releases):

```bash
curl -s https://api.github.com/repos/CLFML/ROS2_Audio_Tools/releases/latest \
  | grep "browser_download_url.*deb" \
  | cut -d : -f 2,3 \
  | tr -d \" \
  | wget -qi -
sudo dpkg -i ./ros-jazzy-audio-tools*.deb
```

---

## ✅ Run the Nodes

Make sure ROS is sourced:

```bash
source /opt/ros/jazzy/setup.sh
```

### Capture Audio

```bash
ros2 run audio_tools audio_capture_node
```

### Playback Audio

```bash
ros2 run audio_tools audio_playback_node \
  --ros-args -p audio_topic:=/audio_stamped
```

---

## 🔁 Full Loopback Example

Start both nodes:

```bash
# Terminal 1
ros2 run audio_tools audio_capture_node

# Terminal 2
ros2 run audio_tools audio_playback_node \
  --ros-args -p audio_topic:=/audio_stamped
```

---

Let me know if you'd like this wrapped into a script or included in a full install guide!