# ubx_ros

## Dependencies

```shell
pip3 install pyubx2 transforms3d
rosdep install --from-paths src -r -y
```

## Usage

### Env Vars

```shell
export NTRIP_HOST=
export NTRIP_PORT=
export NTRIP_MOUNTPOINT=
export NTRIP_USER=
export NTRIP_PASSWORD=
```

### Run

```shell
ros2 launch ubx_ros ubx_ros.launch.py
```

## Acknowledgments

Supporting Extensive Livestock Farming with the use of Autonomous Intelligent Robots

Grant TED2021-132356B-I00 funded by MCIN/AEI/10.13039/501100011033 and by the “European Union NextGenerationEU/PRTR"

<p align="center">
    <img src="https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/robotics_wolf_minimal.png" width="13.5%" /> <img src="https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/micin-financiadoUEnextgeneration-prtr-aei.png" width="60%" />
</p>
