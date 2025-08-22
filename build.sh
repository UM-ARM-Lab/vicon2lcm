#!/bin/bash

# Build script for LCM Vicon Bridge

set -e

echo "Building LCM Vicon Bridge..."

# Create necessary directories
echo "Creating directories..."
mkdir -p build
mkdir -p vicon_msgs

# Generate Python bindings first (so they're available for import)
echo "Generating Python bindings..."
cd vicon_msgs
for lcmfile in ../lcm_msgs/*.lcm; do
    if [ -f "$lcmfile" ]; then
        lcmname=$(basename "$lcmfile" .lcm)
        echo "  Generating Python bindings for $lcmname..."
        lcm-gen --python "$lcmfile"
    fi
done

# Create __init__.py for Python package
cat > __init__.py << 'EOF'
"""Vicon LCM message types."""
EOF

# Add imports for each generated module
for lcmfile in ../lcm_msgs/*.lcm; do
    if [ -f "$lcmfile" ]; then
        lcmname=$(basename "$lcmfile" .lcm)
        echo "from .${lcmname}_t import ${lcmname}_t" >> __init__.py
    fi
done

cd ..

# Create build directory and build C++ components
echo "Building C++ components..."
cd build

# Configure with CMake
echo "Building LCM Vicon Bridge with simplified message structure..."
cmake ..

# Build
make -j$(nproc)

echo ""
echo "✅ Build complete!"
echo ""
echo "📁 Generated files:"
echo "  - C++ headers: vicon_msgs/*.hpp"
echo "  - Python modules: vicon_msgs/*.py"
echo "  - Executables: build/lcm_vicon_bridge, build/vicon_state_receiver"
echo ""
echo "🚀 Usage:"
echo "  ./build/lcm_vicon_bridge                    # Main Vicon bridge (publishes to VICON_STATE and VICON_TWIST)"
echo "  ./build/vicon_state_receiver                # Combined receiver for both state and twist messages"
echo "  ./build/lcm_vicon_bridge --help            # Show all options"
echo ""
echo "The bridge now publishes:"
echo "  - vicon_state_t messages (pose data) on VICON_STATE channel"
echo "  - vicon_twist_t messages (velocity data) on VICON_TWIST channel"
