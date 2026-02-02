# Sara Robot Documentation

Documentation for the Sara Robot (QIHAN S1-W1) with custom Python scripts.

## Table of Contents
- [Overview](#overview)
- [Git Installation](#git-installation)
- [Installation & Setup (go here for install)](#installation--setup)
- [Astra Camera Setup](#astra-camera-setup)
- [Remote Desktop Access](#remote-desktop-access)
- [Troubleshooting](#troubleshooting)
- [Forking the Repository](#forking-the-repository)
- [SSH Connection](#ssh-connection)

## Overview

The Sara Robot has been modified with custom Python scripts located in the `Tag V3.0` directory. The robot includes an Astra camera, with viewing scripts available in the `OpenCV` directory.

**Repository:** https://github.com/tommmie20000/saraJvB-main

## Git Installation

### Linux (Raspberry Pi)
```bash
sudo apt update
sudo apt install -y git
```

### Windows
Using winget:
```bat
winget install --id Git.Git -e --source winget
```

Or download directly from: https://git-scm.com/install/windows

## Installation & Setup

### Complete Installation (Recommended)

```bash
sudo apt update
sudo apt install -y python3 python3-tk git

git clone https://github.com/tommmie20000/saraJvB-main.git
cd saraJvB-main

# Install pyenv
curl https://pyenv.run | bash

# Add to ~/.bashrc:
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init --path)"
eval "$(pyenv init -)"

# Reload bash
source ~/.bashrc

# Install Python 3.10 (required for Astra SDK)
pyenv install 3.10.13
pyenv shell 3.10.13

# Create and activate virtual environment
python3 -m venv ~/astra-venv
source ~/astra-venv/bin/activate

# Install Python packages
pip install -r requirements.txt
```

### If Virtual Environment Creation Fails

If you encounter an error about missing packages:

```bash
sudo apt install python3.13-venv
```

**Note:** Check the error message for the exact Python version needed, as it may differ.

Then remove the failed venv and try again:
```bash
rm -rf venv
python3 -m venv venv
```

## Astra Camera Setup

The Astra camera requires the AstraSDK drivers for communication.

### Prerequisites Installation

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev python3-venv \
libusb-1.0-0-dev libudev-dev wget curl git unzip \
libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev \
libncurses-dev xz-utils tk-dev libffi-dev liblzma-dev
```

### Extract AstraSDK

**For Raspberry Pi:**
```bash
unzip OpenCV/CameraRaspberry/AstraSDK-v2.1.3-Linux-arm.zip

tar -xvf OpenCV/CameraRaspberry/AstraSDK-v2.1.3-Linux-arm/AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64.tar.gz -d OpenCV/CameraRaspberry
```

**For Linux/Ubuntu (x86_64):**
```bash
unzip OpenCV/CameraUbuntu/AstraSDK-v2.1.3-Ubuntu-x86_64.zip -d OpenCV/CameraRaspberry

tar -xvf OpenCV/CameraUbuntu/AstraSDK-v2.1.3-Ubuntu-x86_64/AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64.tar.gz
```

### Install USB Rules

```bash
cp -r AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64 AstraSDK-v2.1.3-94
cd AstraSDK-v2.1.3-94/install
chmod +x install.sh
sudo ./install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Add User to Video Group

```bash
sudo usermod -aG video $USER
```

**Important:** Log out and log back in for permissions to take effect.

### Python Configuration

In your Python script, initialize OpenNI2 with the correct path:

```python
from openni import openni2

openni2.initialize("/path/to/AstraSDK-v2.1.3-94/lib/Plugins/openni2/")
```

Adjust the path to match your AstraSDK extraction location.

### Running Camera Scripts

```bash
python 01_camera.py
```

**Note:** If no Astra camera is connected, you'll see: `DeviceOpen using default: no devices found` — this is expected.

## Troubleshooting

### COM Port Issues (Linux)

If the robot isn't responding, try swapping COM ports in `Tag_V3.0/Common/sara_ports.py`:

```python
COM_HEAD_LINUX: str = "/dev/ttyACM1"
COM_BODY_LINUX: str = "/dev/ttyACM0"
```

Try swapping these values (ACM0 ↔ ACM1).

### COM Port Issues (Windows)

The default ports are COM10/COM11. If your ports differ, update them in `sara_ports.py` rather than changing your system configuration.

### Camera Not Working

1. Verify the Astra camera is connected
2. Check USB permissions (ensure user is in video group)
3. Verify OpenNI2 initialization path is correct
4. Check that Python 3.10 is being used (required for SDK compatibility)

## Additional Resources

- **Astra SDK Downloads:** https://www.orbbec.com/developers/astra-sdk/
- **Repository Issues:** Report problems at https://github.com/tommmie20000/saraJvB-main/issues


## Forking the Repository !! this is optional, you can also just clone the repo and work in there, forking just allows you to upload it to github again.

To create your own version of the Sara Robot code:

1. Navigate to: https://github.com/tommmie20000/saraJvB-main
2. Log in to your GitHub account
3. Click the **Fork** button (top right)
4. Select your account as the destination

### Clone Your Fork Locally

```bash
git clone https://github.com/<your-username>/saraJvB-main.git
cd saraJvB-main
```

### Making Changes

1. Navigate to your cloned folder
2. Make your code changes
3. Stage changes:
```bash
git add .
```
4. Commit changes:
```bash
git commit -m "Description of your changes"
```
5. Push to your fork:
```bash
git push origin main
```

**Tip:** Consider using VS Code with GitHub integration for easier code management and syncing.

### Staying Up-to-Date

To keep your fork synchronized with the original repository:

1. Add the original repo as upstream:
```bash
git remote add upstream https://github.com/tommmie20000/saraJvB-main.git
```

2. Fetch latest changes:
```bash
git fetch upstream
```

3. Merge changes into your local branch:
```bash
git merge upstream/main
```

4. Push updates to your fork:
```bash
git push origin main
```

## SSH Connection

### Windows

1. Connect the Raspberry Pi 5 to your laptop's hotspot
2. Find the Pi's IP address in your hotspot settings under "Connected devices"
3. Open terminal and connect:
```bat
ssh sarajvb@<ip-address>
```
4. Enter the password when prompted

### Linux (Ubuntu)

1. Connect the Raspberry Pi 5 to your laptop's hotspot
2. Find the Pi's IP address:
```bash
arp -a
```
Look for an IP starting with `10.` (or similar, depending on your configuration)

3. Connect via SSH:
```bash
ssh sarajvb@<ip-address>
```
4. Enter the password when prompted

---

*This documentation is primarily in English with some Dutch references maintained for compatibility with existing scripts.*