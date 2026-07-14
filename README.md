<div align="center">

# SysNav: Multi-Level Systematic Cooperation Enables Real-World, Cross-Embodiment Object Navigation

[Haokun Zhu](https://zwandering.github.io/)\*,
[Zongtai Li](https://github.com/igzat1no),
[Zihan Liu](https://zihan-liu.replit.app/),
[Kevin Guo](https://sites.google.com/nyu.edu/kevinguos-profolio/),
[Zhengzhi Lin](https://www.linkedin.com/in/alexlin416/),
[Yuxin Cai](https://yuxin916.github.io/),
[Guofei Chen](https://gfchen01.cc/),
[Chen Lv](https://scholar.google.com/citations?user=UKVs2CEAAAAJ&hl=en),
[Wenshan Wang](http://www.wangwenshan.com/),
[Jean Oh](https://www.cs.cmu.edu/~jeanoh/),
[Ji Zhang](https://frc.ri.cmu.edu/~zhangji/)

Carnegie Mellon University, New York University, Nanyang Technological University

[[Project Page](https://cmu-vln.github.io/)] [[arXiv](https://arxiv.org/abs/2603.06914)]

<img src="img/teaser.jpg" width="100%"/>

</div>

## News

- **[2026-03]** Paper released on [arXiv](https://arxiv.org/abs/2603.06914).
- **[2026-03]** [Project page](https://cmu-vln.github.io/) is online.
- **[2026-04]** Code released for Unity simulation, wheeled robot, Unitree Go2, and Unitree G1 platforms.

## Abstract

Object navigation in real-world environments remains a significant challenge in embodied AI. We present **SysNav**, a three-level object navigation system that decouples semantic reasoning, navigation planning, and motion control. The framework employs Vision-Language Models for high-level semantic guidance and implements a hierarchical room-based navigation strategy that treats rooms as minimal decision-making units, combined with classical exploration for in-room navigation. Through 190 real-world experiments across three robot embodiments (wheeled, quadruped, humanoid), we demonstrate 4-5x improvement in navigation efficiency over existing baselines. The system also achieves state-of-the-art results on HM3D-v1, HM3D-v2, MP3D, and HM3D-OVON simulation benchmarks.

## Demo

### Long-range Object Navigation

<table>
<tr>
<td align="center" width="33%">
<a href="https://www.youtube.com/watch?v=FpF6IATXWds">
<img src="https://img.youtube.com/vi/FpF6IATXWds/maxresdefault.jpg" alt="Find Refrigerator in Lounge" width="100%"/>
</a>
<br><b>Find Refrigerator<br>in Lounge.</b>
<br><a href="https://www.youtube.com/watch?v=FpF6IATXWds">&#9654; Watch on YouTube</a>
</td>
<td align="center" width="33%">
<a href="https://www.youtube.com/watch?v=GqRUvwAEqc8">
<img src="https://img.youtube.com/vi/GqRUvwAEqc8/maxresdefault.jpg" alt="Find Blue Trash Can in Classroom" width="100%"/>
</a>
<br><b>Find Blue Trash Can<br>in Classroom.</b>
<br><a href="https://www.youtube.com/watch?v=GqRUvwAEqc8">&#9654; Watch on YouTube</a>
</td>
<td align="center" width="33%">
<a href="https://www.youtube.com/watch?v=A78TSwI78iM">
<img src="https://img.youtube.com/vi/A78TSwI78iM/maxresdefault.jpg" alt="Find Microwave Oven near Refrigerator" width="100%"/>
</a>
<br><b>Find Microwave Oven<br>near Refrigerator.</b>
<br><a href="https://www.youtube.com/watch?v=A78TSwI78iM">&#9654; Watch on YouTube</a>
</td>
</tr>
</table>

### Cross-Embodiment Object Navigation

<table>
<tr>
<th></th>
<th align="center">System View</th>
<th align="center">Third-person View</th>
</tr>
<tr>
<td rowspan="2" align="center" width="12%"><b>Wheeled<br>Robot</b></td>
<td width="44%">

[.webm](https://github.com/user-attachments/assets/bd7cec26-9198-401a-8ff6-d7f3f8f6f093)

</td>
<td width="44%">

[.webm](https://github.com/user-attachments/assets/8821366c-b439-4661-8802-200c9259f933)

</td>
</tr>
<tr>
<td colspan="2" align="center"><em>Find the microwave_oven.</em></td>
</tr>
<tr>
<td rowspan="2" align="center"><b>Quadruped<br>(Go2)</b></td>
<td>

[.webm](https://github.com/user-attachments/assets/428ad7a1-82f8-4f3b-88bd-6ab620c707ea)

</td>
<td>

[.webm](https://github.com/user-attachments/assets/09abb740-46ce-405a-8922-697fc074fcaf)

</td>
</tr>
<tr>
<td colspan="2" align="center"><em>Find the blue trash_can.</em></td>
</tr>
<tr>
<td rowspan="2" align="center"><b>Humanoid<br>(G1)</b></td>
<td>

[.webm](https://github.com/user-attachments/assets/b235e61d-f2b7-4300-b982-33567c5fa880)

</td>
<td>

[.webm](https://github.com/user-attachments/assets/ad1b03a0-b41c-493c-a549-cff6a232cac1)

</td>
</tr>
<tr>
<td colspan="2" align="center"><em>Find the tv_monitor on the black desk.</em></td>
</tr>
</table>

<p align="center"><em>More demos on our <a href="https://cmu-vln.github.io/">project page</a>.</em></p>

## Platforms

This repository supports three robot embodiments, each maintained on its own branch. Switch to the corresponding branch (`git checkout unitree_go2` / `git checkout unitree_g1`) before building and running on a Unitree robot.

### Wheeled Robot + Unity simulation &mdash; [`main`](https://github.com/zwandering/VLM_ROS) *(you are here)*

- Custom wheeled vehicle with Mecanum wheels (indoor carpet) or standard wheels (hard floor / outdoors)
- Livox Mid-360 lidar + Ricoh Theta Z1 360-degree camera
- Motor controller connected via USB serial (`/dev/ttyACM0` by default)
- Gaming laptop (RTX 4090) as the processing computer
- PS3/Xbox-style joystick for teleoperation

Detailed hardware photos and assembly info: [Real-robot Setup &rarr; Hardware](#hardware).

### Unitree Go2 Quadruped &mdash; [`unitree_go2`](https://github.com/zwandering/VLM_ROS/tree/unitree_go2)

- Unitree Go2 quadruped, controlled via WebRTC
- Livox Mid-360 lidar + Ricoh Theta Z1 360-degree camera
- Asus NUC 14 Pro (Intel Core Ultra 5) as the onboard computer
- Desktop workstation / Laptop with NVIDIA RTX 4090 for the semantic mapping and VLM reasoning
- Wired / WiFi network shared between robot, NUC, and desktop

### Unitree G1 Humanoid &mdash; [`unitree_g1`](https://github.com/zwandering/VLM_ROS/tree/unitree_g1)

- Unitree G1 humanoid, controlled via WebRTC
- Livox Mid-360 lidar + Ricoh Theta Z1 360-degree camera
- Asus NUC 14 Pro (Intel Core Ultra 5) as the onboard computer
- Desktop workstation / Laptop with NVIDIA RTX 4090 for the semantic mapping and VLM reasoning
- Wired / WiFi network shared between robot, NUC, and desktop

## This Branch: Scene-Graph Pipeline Only

> This branch (`deepclean`) is the **scene-graph-construction-only** reduction of
> SysNav. Navigation execution (the TARE steering outputs, `base_autonomy`,
> `route_planner`) and the in-repo SLAM (`arise_slam`) have been removed — the
> robot is driven by its own onboard planner (or teleop / bag replay), and the
> pipeline passively consumes the robot/bag's registered cloud + odometry
> (`/<ns>/cloud_registered` + `/<ns>/lio/odometry`) plus the camera to build and
> export a GADM-style scene-graph JSON. Start with
> [`ARCHITECTURE.md`](ARCHITECTURE.md) for the developer guide; the full
> object-navigation system described by the paper lives on the original branches.

## Contents

- [Demo](#demo)
- [Platforms](#platforms)
- [Installation](#installation)
  - [Dependencies](#1-dependencies)
  - [Submodules and Python Packages](#2-submodules-and-python-packages)
  - [Compile](#3-compile)
  - [VLM API Key](#vlm-api-key)
- [Running the Scene-Graph Pipeline](#running-the-scene-graph-pipeline)
  - [Bag Replay (tmux)](#bag-replay-tmux)
  - [Docker (bag-direct / live / demo)](#docker-bag-direct--live--demo)
  - [Output](#output)
- [Credits](#credits)
- [Citation](#citation)
- [License](#license)

## Installation

The system has been tested on **Ubuntu 24.04** with **ROS2 Jazzy**.

### 1) Dependencies

Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html), then:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install system dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-desktop-full ros-jazzy-pcl-ros libpcl-dev git
sudo apt install -y nlohmann-json3-dev
sudo apt install ros-jazzy-backward-ros
```

### 2) Submodules and Python Packages

```bash
git submodule update --init --recursive

pip install -r requirement.txt --break-system-package

# detectron2
python -m pip install 'git+https://github.com/facebookresearch/detectron2.git' --break-system-package

# pytorch3d
pip install "git+https://github.com/facebookresearch/pytorch3d.git" --no-build-isolation --break-system-package

# sam2
cd src/semantic_mapping/semantic_mapping/external/sam2
pip install -e . --break-system-package
cd checkpoints && ./download_ckpts.sh && cd ../..

# spacy
python -m spacy download en_core_web_sm --break-system-package

# CLIP
pip install git+https://github.com/ultralytics/CLIP.git --break-system-package

# YOLO models
python set_yolo_e.py
python set_yolo_world.py
```

### 3) Compile

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> Both flags matter: the workspace convention is `Release` (the planner is
> unusably slow at `-O0`, which is what an empty build type gives you), and
> `--symlink-install` is required — `tare_planner` installs only its executables
> and resolves its shared libraries through the build tree.

### VLM API Key

The VLM node supports two providers via the OpenAI-compatible interface. Set **one** of the following:

**Gemini** (default) &mdash; get a key from [Google AI Studio](https://aistudio.google.com/app/api-keys):
```bash
export GEMINI_API_KEY="your-api-key-here"
```

**Qwen (DashScope)** &mdash; get a key from [Alibaba Cloud DashScope](https://dashscope.console.aliyun.com/):
```bash
export DASHSCOPE_API_KEY="your-api-key-here"
```

If both keys are set, Gemini is used by default; override with `export VLM_PROVIDER=qwen`. Optionally override Qwen model names with `QWEN_MODEL` / `QWEN_MODEL_LITE`. Add the line(s) to `~/.bashrc` so they persist across terminal sessions.

## Running the Scene-Graph Pipeline

Everything is brought up by **one launch file** — `tare_planner scene_graph.launch`
— which starts the `<ns>/odom → map` static tf, detection + semantic mapping
(`objects:=true`), room segmentation, the VLM node, the planner node (scene-graph
builder + exporter), and RViz. The **single per-robot knob** is
`robot_namespace` in
[`src/exploration_planner/tare_planner/config/robot.yaml`](src/exploration_planner/tare_planner/config/robot.yaml):
every node composes its inputs as `/<robot_namespace>/cloud_registered`,
`/<robot_namespace>/lio/odometry`, `/<robot_namespace>/camera/...`; camera
intrinsics come from `camera_info` and extrinsics from tf. Set it to match the
robot (or the bag's recording robot) and nothing else needs editing.

### Bag Replay (tmux)

```bash
tmuxp load vlm_ros_alphaz_bag_direct.tmuxp.yaml
```

Edit the bag path / start offset inside that yaml (per-run knobs live there, and
only there). When launching by hand instead, note two gotchas: play the bag
**paused with `--clock` before starting the stack** (the planner exits if its
first tick sees sim time 0), and pass `decompress_camera:=true` for bags that
carry only the compressed camera topic.

### Docker (bag-direct / live / demo)

The containerized flow (build-in-volume, source bind-mounted) is documented in
[`docker/README.md`](docker/README.md):

```bash
docker/run.sh build                                   # one-time compile
MODE=bag-direct BAG=/path/to/recording docker/run.sh  # replay a bag
MODE=live  ROBOT_IP=... LAPTOP_IP=... docker/run.sh   # live robot via ros1_bridge
MODE=demo  ROBOT_IP=... LAPTOP_IP=... docker/run.sh   # robot-gated demo runs
```

### Output

Scene-graph snapshots are written under `output/scene_graph/run_<timestamp>/` as
`snapshot_<n>_<t>.json` (periodic / manual via publishing `ssg` on
`/keyboard_input`) and `snapshot_final.json` (end-of-bag watchdog). The JSON
schema and export configuration are documented in
[`scene_graph_exporter/README.md`](src/exploration_planner/tare_planner/src/scene_graph_exporter/README.md)
and [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Credits

The project is led by [Ji Zhang's](https://frc.ri.cmu.edu/~zhangji) group at Carnegie Mellon University.

The scene-graph builder grew out of the [TARE planner](https://github.com/caochao39/tare_planner) and the [Autonomous Exploration Development Environment](https://www.cmu-exploration.com).

## Citation

If you find this work useful, please consider citing:
```bibtex
@article{zhu2026sysnav,
  title={SysNav: Multi-Level Systematic Cooperation Enables Real-World, Cross-Embodiment Object Navigation},
  author={Zhu, Haokun and Li, Zongtai and Liu, Zihan and Guo, Kevin and Lin, Zhengzhi and Cai, Yuxin and Chen, Guofei and Lv, Chen and Wang, Wenshan and Oh, Jean and Zhang, Ji},
  journal={arXiv preprint arXiv:2603.06914},
  year={2026}
}
```

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).

Some third-party packages retain their original open-source licenses (BSD, MIT, Apache 2.0, GPLv3). See individual `package.xml` files for per-package license declarations.
