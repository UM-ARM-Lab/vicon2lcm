#!/bin/bash

# Build script for LCM Vicon Bridge

set -e

echo "Building LCM Vicon Bridge..."

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Building LCM Vicon Bridge with simplified message structure..."
cmake ..

# Build
make -j$(nproc)

echo "Build complete!"
echo ""
echo "Usage:"
echo "  ./lcm_vicon_bridge                    # Main Vicon bridge (publishes to VICON_STATE and VICON_TWIST)"
echo "  ./vicon_state_receiver                # Combined receiver for both state and twist messages"
echo "  ./lcm_vicon_bridge --help            # Show all options"
echo ""
echo "The bridge now publishes:"
echo "  - vicon_state_t messages (pose data) on VICON_STATE channel"
echo "  - vicon_twist_t messages (velocity data) on VICON_TWIST channel"
