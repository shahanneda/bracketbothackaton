#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e
# Update package lists
echo "Updating package lists..."
sudo apt update
# Install and enable SSH service
echo "Installing and enabling SSH service..."
sudo apt install -y ssh
sudo systemctl enable ssh
sudo systemctl start ssh
# Check SSH service status
echo "Checking SSH service status..."
sudo SYSTEMD_PAGER='' systemctl status ssh
# Install Python development packages
echo "Installing Python development packages..."
sudo apt install -y python3-dev python3-pip python3-venv
# Create a Python virtual environment in the home directory
echo "Creating a Python virtual environment..."
cd ~
python3 -m venv .venv
# Activate the virtual environment
echo "Activating the virtual environment..."
source ~/.venv/bin/activate
# Make the virtual environment activate automatically in new terminals
echo "Configuring automatic virtual environment activation..."
echo 'source ~/.venv/bin/activate' >> ~/.bashrc
# Install RPi.GPIO and lgpio libraries inside the virtual environment
echo "Installing RPi.GPIO and lgpio libraries..."
pip install RPi.GPIO rpi-lgpio
# Install Adafruit MPU6050 library
echo "Installing Adafruit MPU6050 library..."
pip install adafruit-circuitpython-mpu6050

# Install mosquitto broker and clients
echo "Installing mosquitto broker and clients..."
sudo apt install -y mosquitto mosquitto-clients ufw
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

	@@ -89,26 +89,13 @@ sudo usermod -a -G dialout,audio $USER
echo "Configuring boot settings..."
sudo sh -c 'printf "\ndisable_poe_fan=1\nenable_uart=1\ndtoverlay=uart1-pi5\ndtparam=i2c_arm=on\ndtoverlay=i2c1\n" >> /boot/firmware/config.txt'

# Install git
# echo "Installing git..."
# sudo apt install -y git

# # Clone the BracketBot quickstart repository
# echo "Cloning the BracketBot quickstart repository..."
# cd ~
# git clone https://github.com/BracketBotCapstone/quickstart
# cd quickstart

# Install required Python packages
echo "Installing required Python packages..."
pip install numpy sympy control matplotlib pyserial

# Inform the user about the calibration step
echo -e "\e[1;33mPlease ensure the ODrive is connected via USB before proceeding.\e[0m"
read -p "Press Enter once the ODrive is connected..."

# Run the calibration script
echo "Running the ODrive calibration script..."
python3 ~/quickstart/setup/calibrate_odrive.py

echo "Setup complete! You may need to reboot your system for all changes to take effect."