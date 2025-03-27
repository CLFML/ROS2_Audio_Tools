# Getting started with Pixi

Pixi makes cross-platform ROS 2 development easy. You can build and run both capture and playback nodes on **Linux and Windows**—with no system-wide ROS install.

---

## 📦 Install Pixi

**Linux**:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

**Windows** (PowerShell):

```powershell
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
```

---

## 🚀 Clone & Build Project

```bash
git clone https://github.com/CLFML/ROS2_Audio_Tools.git
cd ROS2_Audio_Tools
pixi install
pixi run build
```

Or launch VSCode with the environment:

```bash
pixi run vscode
```

> ✅ **Note (Windows):** Always build in **Release** or **RelWithDebInfo**, not Debug!  
> *(Ctrl+Shift+P → "CMake: Select Variant")*

---

## ⚡ Using as a Pixi Dependency

Want to use `audio_tools` from another Pixi-based project?

### 1. Init a new project

```bash
mkdir my_audio_project && cd my_audio_project
pixi init
```

### 2. Edit `pixi.toml`

Add these:

```toml
[project]
channels = [
  "https://fast.prefix.dev/conda-forge",
  "https://prefix.dev/robostack-jazzy",
  "https://clfml.github.io/conda_ros2_jazzy_channel/"
]

[dependencies]
ros-jazzy-ros-base = "*"
ros-jazzy-audio-tools = "*"
colcon-common-extensions = "*"
rosdep = "*"
```

### 🧠 Optional: VSCode Support

Add to your `pixi.toml`:

```toml
[target.linux-64.dependencies]
python-devtools = "*"
pybind11 = "*"
numpy = "*"

[target.win-64.dependencies]
python-devtools = "*"

[target.linux-64.tasks]
vscode = 'env -u LD_LIBRARY_PATH code .'

[target.win-64.tasks]
vscode = "code ."
```

---

### 3. Run the nodes

```bash
pixi install
pixi run ros2 run audio_tools audio_capture_node
```

---


## ▶️ Example: Full Audio Loopback

Launch both nodes using ROS 2 launch (It is really advised to use headphones!):

```bash
pixi run build
pixi shell
# Windows:
.\install\setup.bat
# Linux:
source install/setup.sh


```
