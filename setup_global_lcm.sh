#!/usr/bin/env bash

# Setup global LCM environment for custom types
# Run this script once to configure your system

echo "Setting up global LCM environment for custom Vicon types..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create user LCM directory if it doesn't exist
mkdir -p ~/.local/share/java

# Generate Java LCM types
echo "Generating Java LCM types..."
lcm-gen --java "$SCRIPT_DIR/lcm_msgs/vicon_state.lcm"
lcm-gen --java "$SCRIPT_DIR/lcm_msgs/twist3D.lcm"

# Compile Java classes
echo "Compiling Java classes..."
javac -cp "/usr/share/java/lcm.jar" "$SCRIPT_DIR/lcm_msgs/vicon_msgs/*.java"

# Create JAR file
echo "Creating JAR file..."
jar cf "$SCRIPT_DIR/lcm_msgs/vicon_types.jar" -C "$SCRIPT_DIR/lcm_msgs/" vicon_msgs

# Copy JAR file to user LCM directory
cp "$SCRIPT_DIR/lcm_msgs/vicon_types.jar" ~/.local/share/java/

# Clean up generated files (keep only source .lcm files)
echo "Cleaning up generated files..."
rm -rf "$SCRIPT_DIR/lcm_msgs/vicon_msgs"
rm -f "$SCRIPT_DIR/lcm_msgs/vicon_types.jar"

# Add to your shell profile (bash/zsh)
SHELL_PROFILE=""
if [[ -f ~/.bashrc ]]; then
    SHELL_PROFILE=~/.bashrc
elif [[ -f ~/.zshrc ]]; then
    SHELL_PROFILE=~/.zshrc
else
    SHELL_PROFILE=~/.profile
fi

# Check if CLASSPATH is already set
if ! grep -q "vicon_types.jar" "$SHELL_PROFILE"; then
    echo "" >> "$SHELL_PROFILE"
    echo "# LCM Custom Types" >> "$SHELL_PROFILE"
    echo "export CLASSPATH=\"\$HOME/.local/share/java/vicon_types.jar:\$CLASSPATH\"" >> "$SHELL_PROFILE"
    echo "Added CLASSPATH to $SHELL_PROFILE"
else
    echo "CLASSPATH already configured in $SHELL_PROFILE"
fi

# Also add to current session
export CLASSPATH="$HOME/.local/share/java/vicon_types.jar:$CLASSPATH"

echo ""
echo "✅ Global LCM setup complete!"
echo "Your custom Vicon types are now available to any LCM tool on your system."
echo ""
echo "To use in current session:"
echo "  source $SHELL_PROFILE"
echo ""
echo "Or restart your terminal."
echo ""
echo "Test with: lcm-spy"
