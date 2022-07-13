# ubx_ros

## Dependencies

```shell
pip3 install pyubx2
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
