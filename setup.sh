#!/usr/bin/env bash
# Setup script for multi-drone testbed on Ubuntu 24.04 (ROS2 Jazzy + Crazyswarm2)
# Run once after cloning the repo: bash setup.sh

set -e
echo "=== Multi-Drone Testbed Setup ==="

# ── 1. ROS2 Jazzy ─────────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo "[1/5] Installing ROS2 Jazzy..."
    sudo apt update && sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-jazzy-ros-base ros-jazzy-rviz2 \
        python3-colcon-common-extensions python3-rosdep python3-pip
    sudo rosdep init || true
    rosdep update
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source /opt/ros/jazzy/setup.bash
else
    echo "[1/5] ROS2 already installed, skipping."
    source /opt/ros/jazzy/setup.bash
fi

# ── 2. Crazyswarm2 via apt ────────────────────────────────────────────────────
echo "[2/5] Installing Crazyswarm2 (apt)..."
sudo apt install -y ros-jazzy-crazyflie
pip3 install cflib nicegui
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ── 3. Crazyradio USB permissions ─────────────────────────────────────────────
echo "[3/5] Setting up Crazyradio USB permissions..."
if [ ! -f /etc/udev/rules.d/99-crazyradio.rules ]; then
    sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", MODE=\"0664\", GROUP=\"plugdev\"" > /etc/udev/rules.d/99-crazyradio.rules'
    sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE=\"0664\", GROUP=\"plugdev\"" >> /etc/udev/rules.d/99-crazyradio.rules'
    sudo udevadm control --reload-rules
    sudo usermod -aG plugdev $USER
    echo "  NOTE: Log out and back in (or reboot) for USB group change to take effect."
fi

# ── 4. Build the drone_testbed ROS2 package ───────────────────────────────────
echo "[4/5] Building drone_testbed package..."
cd "$(dirname "$0")/ros2_ws"
colcon build --symlink-install
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc

# ── 5. Python deps for standalone scripts ─────────────────────────────────────
echo "[5/5] Installing Python dependencies..."
pip3 install cflib numpy matplotlib pyyaml

echo ""
echo "=== Setup complete! ==="
echo ""
echo "NEXT STEPS:"
echo "  1. Edit config/crazyflies.yaml — set your drone's radio URI and VICON body name"
echo "  2. Edit config/motion_capture.yaml — set the VICON PC's IP address"
echo "  3. Plug in the Crazyradio dongle"
echo "  4. Run the simple flight test:"
echo "     source ~/.bashrc"
echo "     bash run_hardware_test.sh"
echo ""
echo "See README.md for full details."
