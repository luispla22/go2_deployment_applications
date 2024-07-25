#!/bin/bash

set -e

# Remote machine details
REMOTE_USER="unitree"
REMOTE_IP="192.168.123.18"
REMOTE_PASS="123"

# Local temporary directory
LOCAL_TMP_DIR="./sdk_tmp"

# Make sure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Please install it first."
    exit 1
fi

# Function to run commands on remote machine
run_remote_command() {
    sshpass -p $REMOTE_PASS ssh -tt $REMOTE_USER@$REMOTE_IP "$1"
}

# Function to transfer files to remote machine
transfer_to_remote() {
    sshpass -p $REMOTE_PASS scp -r $1 $REMOTE_USER@$REMOTE_IP:$2
}

# Create temporary directory and clone repositories locally
echo "Cloning repositories locally..."
mkdir -p $LOCAL_TMP_DIR
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git -b releases/0.10.x $LOCAL_TMP_DIR/cyclonedds
git clone https://github.com/eclipse-cyclonedds/cyclonedds-python.git $LOCAL_TMP_DIR/cyclonedds-python
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git $LOCAL_TMP_DIR/unitree_sdk2_python

# Transfer files to remote machine
echo "Transferring files to remote machine..."
transfer_to_remote $LOCAL_TMP_DIR ~/sdk_tmp

# Build CycloneDDS and install CycloneDDS Python
echo "Building CycloneDDS and installing CycloneDDS Python on remote machine..."
run_remote_command "
    cd ~/sdk_tmp/cyclonedds
    mkdir -p build install
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=../install
    cmake --build . --config RelWithDebInfo --target install
    cd ..
    export CYCLONEDDS_HOME=\$(pwd)/install
    cd ~/sdk_tmp/cyclonedds-python
    pip3 install .
"

# Install Unitree SDK 2 Python on remote machine
echo "Installing Unitree SDK 2 Python on remote machine..."
run_remote_command "
    export CYCLONEDDS_HOME=~/sdk_tmp/cyclonedds/install
    cd ~/sdk_tmp/unitree_sdk2_python
    pip3 install -e .
"

# Clean up local temporary directory
rm -rf $LOCAL_TMP_DIR

echo "Installation complete!"

# Verify installations
echo "Verifying CycloneDDS installation..."
run_remote_command "ls ~/sdk_tmp/cyclonedds/install/bin/cyclonedds" && echo "CycloneDDS installed successfully" || echo "CycloneDDS installation failed"

echo "Verifying CycloneDDS Python installation..."
run_remote_command "python3 -c \"import cyclonedds; print('CycloneDDS Python installed successfully')\"" || echo "CycloneDDS Python installation failed"

echo "Verifying Unitree SDK 2 Python installation..."
run_remote_command "export CYCLONEDDS_HOME=~/sdk_tmp/cyclonedds/install && python3 -c \"import unitree_sdk2py; print('Unitree SDK 2 Python installed successfully')\"" || echo "Unitree SDK 2 Python installation failed"

# Add CYCLONEDDS_HOME to .bashrc for persistence
echo "Adding CYCLONEDDS_HOME to .bashrc..."
run_remote_command "echo 'export CYCLONEDDS_HOME=~/sdk_tmp/cyclonedds/install' >> ~/.bashrc"

echo "Setup complete. Please log out and log back in, or run 'source ~/.bashrc' to apply the changes."