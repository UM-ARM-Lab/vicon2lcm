# LCM Vicon Bridge

A lightweight Vicon motion capture bridge that publishes Vicon data via LCM (Lightweight Communications and Marshalling).

## Message Types

### VICON_STATE Channel
Publishes `vicon_state_t` messages containing:
- Position (x, y, z) in meters
- Orientation as quaternions (w, x, y, z)
- Roll, Pitch, Yaw in radians
- Occlusion status for each body
- Frame ID and tracker information

### VICON_TWIST Channel
Publishes `vicon_twist_t` messages containing:
- Linear velocity (vx, vy, vz) in m/s
- Angular velocity (wx, wy, wz) in rad/s
- Velocity estimates computed via finite differencing
- One velocity estimate per tracked body/segment

## Build

```bash
./build.sh
```

This will:
1. Generate LCM C++ headers from `.lcm` message definitions
2. Build the Vicon bridge and receiver executables
3. Install the binaries

## Usage

### Main Vicon Bridge
```bash
# Default settings (10.10.10.5:801, ServerPush mode)
./build/lcm_vicon_bridge

# Custom settings
./build/lcm_vicon_bridge --host 192.168.1.100 --port 801 --mode ClientPullPreFetch
```

### Combined State & Twist Receiver
```bash
./build/vicon_state_receiver
```

This receiver displays both pose and velocity data from both channels:
- **VICON_STATE**: Position, orientation, RPY angles, occlusion status
- **VICON_TWIST**: Linear and angular velocities computed via finite differencing

## Command Line Options

- `--host HOST`: Vicon server hostname (default: 10.10.10.5)
- `--port PORT`: Vicon server port (default: 801)
- `--channel CH`: LCM channel name (default: VICON_STATE)
- `--mode MODE`: Streaming mode: ServerPush/ClientPull/ClientPullPreFetch (default: ServerPush)
- `--help`: Show help message





