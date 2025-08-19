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
echo "  ./lcm_vicon_bridge                    # Main Vicon bridge (publishes to VICON_STATE)"
echo "  ./vicon_state_receiver                # Receiver for Vicon state messages"
echo "  ./lcm_vicon_bridge --help            # Show all options"
echo ""
echo "The bridge now publishes a single vicon_state_t message containing all tracked bodies."
