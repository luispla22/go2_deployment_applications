# # Local temporary directory
# $LOCAL_TMP_DIR = ".\jetson_packages"

# Path to the script to be transferred
$INSTALL_SCRIPT = ".\scripts\install_python_deps_windows_go2part.sh"

# # File containing list of packages to install
# $PACKAGES_FILE = "packages.txt"

# # Check if packages file exists
# if (-not (Test-Path $PACKAGES_FILE)) {
#     Write-Host "Packages file '$PACKAGES_FILE' not found. Please create it with a list of packages to install."
#     exit 1
# }

# # Create local temporary directory
# New-Item -ItemType Directory -Force -Path $LOCAL_TMP_DIR | Out-Null

# # Generate requirements file
# Write-Host "Generating requirements file..."
# Get-Content $PACKAGES_FILE | Set-Content $LOCAL_TMP_DIR\requirements.txt

# # Download packages and dependencies
# Write-Host "Downloading packages and dependencies for ARM architecture..."
# pip download --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.8 -r $LOCAL_TMP_DIR\requirements.txt --dest $LOCAL_TMP_DIR

# Transfer the jetson_packages directory to the remote machine
scp -r .\jetson_packages unitree@192.168.123.18:/home/unitree

# Transfer the install script to the remote machine
scp .\scripts\install_python_deps_windows_go2part.sh unitree@192.168.123.18:/home/unitree/jetson_packages/

Write-Host "Package download and transfer complete!"
