#!/bin/bash

# Directory containing the transferred packages
PACKAGE_DIR="/home/unitree/jetson_packages"

# Check if the package directory exists
if [ ! -d "$PACKAGE_DIR" ]; then
    echo "Package directory $PACKAGE_DIR not found. Please ensure packages were transferred correctly."
    exit 1
fi

# Install pip if it's not already installed
if ! command -v pip3 &> /dev/null; then
    echo "pip3 not found. Installing pip..."
    sudo apt-get update
    sudo apt-get install -y python3-pip
fi

# Install the packages
echo "Installing packages..."
pip3 install --no-index --find-links="$PACKAGE_DIR" -r "$PACKAGE_DIR/requirements.txt"
echo "Installation complete!"