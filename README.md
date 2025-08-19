# LCM Vicon Bridge

A lightweight bridge that connects Vicon motion capture systems to LCM (Lightweight Communications and Marshalling) using a simplified message structure. Designed for minimal latency and real-time applications.


## Dependencies

- **Required**: LCM, Vicon DataStream SDK
- **Message Types**: Custom `vicon_state_t` (generated from LCM definition)

## Building

```bash
./build.sh
```

This will create two executables:
- `lcm_vicon_bridge` - The main bridge that connects to Vicon and publishes LCM messages
- `vicon_state_receiver` - A simple receiver to test and inspect LCM messages

## Usage

### 1. Start the Vicon Bridge

Connect to your real Vicon system:

```bash
cd build
# Connect to default Vicon server (10.10.10.5:801)
./lcm_vicon_bridge --host YOUR_VICON_IP --port YOUR_VICON_PORT

# Or use defaults (10.10.10.5:801)
./lcm_vicon_bridge
```

### 2. Monitor the Data

In another terminal, use the receiver to see the data:

```bash
cd build
./vicon_state_receiver
```

Or use `lcm-spy` to inspect the raw LCM messages:

```bash
lcm-spy
```

## Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `--host` | Vicon server hostname | 10.10.10.5 |
| `--port` | Vicon server port | 801 |
| `--channel` | LCM channel name | VICON_STATE |
| `--mode` | Streaming mode | ServerPush |

## Message Format

The bridge publishes `vicon_msgs::vicon_state_t` messages containing:

- **`utime`**: Timestamp in microseconds
- **`tracker_name`**: Name of the Vicon tracker
- **`body_names[10]`**: Array of rigid body names
- **`positions[10][3]`**: 3D positions in meters [x, y, z] for each body
- **`quaternions[10][4]`**: Quaternion orientations [w, x, y, z] for each body
- **`rpy[10][3]`**: Roll, pitch, yaw angles (set to 0, using quaternions)
- **`num_bodies`**: Number of active tracked bodies (max 10)

