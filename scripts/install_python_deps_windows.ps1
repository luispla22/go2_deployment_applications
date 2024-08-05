# Remote machine details
$REMOTE_USER = "unitree"
$REMOTE_IP = "192.168.123.18"
$REMOTE_PASS = "123"

# Local temporary directory
$LOCAL_TMP_DIR = ".\jetson_packages"

# Remote temporary directory
$REMOTE_TMP_DIR = "/home/$REMOTE_USER/jetson_packages"

# File containing list of packages to install
$PACKAGES_FILE = "packages.txt"

# Make sure plink and pscp are available
if (-not (Get-Command "plink.exe" -ErrorAction SilentlyContinue) -or -not (Get-Command "pscp.exe" -ErrorAction SilentlyContinue)) {
    Write-Host "plink or pscp is not found. Please ensure PuTTY is installed and both executables are in your PATH."
    exit 1
}

# Check if packages file exists
if (-not (Test-Path $PACKAGES_FILE)) {
    Write-Host "Packages file '$PACKAGES_FILE' not found. Please create it with a list of packages to install."
    exit 1
}

# Create local temporary directory
New-Item -ItemType Directory -Force -Path $LOCAL_TMP_DIR | Out-Null

# Generate requirements file
Write-Host "Generating requirements file..."
Get-Content $PACKAGES_FILE | Set-Content $LOCAL_TMP_DIR\requirements.txt

# Download packages and dependencies
Write-Host "Downloading packages and dependencies for ARM architecture..."
pip download --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.8 -r $LOCAL_TMP_DIR\requirements.txt --dest $LOCAL_TMP_DIR

# Create remote directory
Write-Host "Creating remote directory..."
echo y | plink.exe -ssh -pw $REMOTE_PASS $REMOTE_USER@$REMOTE_IP "mkdir -p $REMOTE_TMP_DIR"

# Transfer files to remote machine
Write-Host "Transferring files to remote machine..."
Get-ChildItem $LOCAL_TMP_DIR | ForEach-Object {
    $file = $_.Name
    Write-Host "Transferring $file..."
    echo y | pscp -pw $REMOTE_PASS $LOCAL_TMP_DIR\$file $REMOTE_USER@$REMOTE_IP`:$REMOTE_TMP_DIR/
}

# Transfer the install script to the remote machine
Write-Host "Transferring the install script to remote machine..."
echo y | pscp -pw $REMOTE_PASS $INSTALL_SCRIPT $REMOTE_USER@$REMOTE_IP`:$REMOTE_TMP_DIR/

Write-Host "Package download and transfer complete!"