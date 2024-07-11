#!/bin/bash

# Remote machine details
REMOTE_USER="unitree"
REMOTE_IP="192.168.123.18"
REMOTE_PASS="123"

# Local temporary directory
LOCAL_TMP_DIR="./jetson_packages"

# File containing list of packages to install
PACKAGES_FILE="packages.txt"

# Make sure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Please install it first."
    exit 1
fi

# Check if packages file exists
if [ ! -f "$PACKAGES_FILE" ]; then
    echo "Packages file '$PACKAGES_FILE' not found. Please create it with a list of packages to install."
    exit 1
fi

# Function to run commands on remote machine
run_remote_command() {
    sshpass -p $REMOTE_PASS ssh $REMOTE_USER@$REMOTE_IP "$1"
}

# Check Python version on remote machine
echo "Checking Python version on remote machine..."
REMOTE_PYTHON_VERSION=$(run_remote_command "python3 --version 2>&1 | cut -d' ' -f2")
echo "Remote Python version: $REMOTE_PYTHON_VERSION"

# Check if pip is installed on remote machine
PIP_INSTALLED=$(run_remote_command "command -v pip3 || echo 'not installed'")

# Create temporary directory
mkdir -p $LOCAL_TMP_DIR

# Generate requirements file with all dependencies
echo "Generating comprehensive requirements file..."
pip3 install pip-tools
pip-compile --generate-hashes --output-file $LOCAL_TMP_DIR/requirements.txt $PACKAGES_FILE

# Add importlib-metadata to requirements if Python version < 3.10
if [[ "${REMOTE_PYTHON_VERSION%.*}" < "3.10" ]]; then
    echo "importlib-metadata" >> $LOCAL_TMP_DIR/requirements.txt
    pip-compile --generate-hashes --output-file $LOCAL_TMP_DIR/requirements.txt $LOCAL_TMP_DIR/requirements.txt
fi

# Download packages and dependencies
echo "Downloading packages and dependencies for ARM architecture..."
pip3 download --only-binary=:all: --platform manylinux2014_aarch64 --python-version ${REMOTE_PYTHON_VERSION%.*} -r $LOCAL_TMP_DIR/requirements.txt --dest $LOCAL_TMP_DIR

# Transfer files to remote machine
echo "Transferring files to remote machine..."
sshpass -p $REMOTE_PASS scp -r $LOCAL_TMP_DIR $REMOTE_USER@$REMOTE_IP:~/

# Install packages on remote machine
echo "Installing packages on remote machine..."
run_remote_command "
    # Install pip if it's not already installed
    if [ '$PIP_INSTALLED' = 'not installed' ]; then
        sudo apt-get update
        sudo apt-get install -y python3-pip
    fi

    # Install the packages
    pip3 install --no-index --find-links=./jetson_packages -r ./jetson_packages/requirements.txt

    # Clean up
    rm -rf ./jetson_packages
"

# Clean up local temporary directory
rm -rf $LOCAL_TMP_DIR

echo "Installation complete!"

# Verify installation
echo "Verifying installation..."
while IFS= read -r package || [[ -n "$package" ]]; do
    # Skip empty lines and comments
    [[ $package =~ ^[[:space:]]*$ || $package =~ ^# ]] && continue
    VERSION=$(run_remote_command "python3 -c \"import $package; print($package.__version__)\"")
    echo "Installed $package version: $VERSION"
done < "$PACKAGES_FILE"

# Also verify importlib-metadata if it was added
if [[ "${REMOTE_PYTHON_VERSION%.*}" < "3.10" ]]; then
    VERSION=$(run_remote_command "python3 -c \"import importlib_metadata; print(importlib_metadata.__version__)\"")
    echo "Installed importlib-metadata version: $VERSION"
fi