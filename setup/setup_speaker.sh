#!/bin/bash

# Exit on error
set -e

echo "Installing speaker dependencies..."

# Update pip
pip3 install --upgrade pip

# Install required Python packages
pip3 install elevenlabs sounddevice soundfile python-dotenv

echo "Speaker setup complete!"

# Test imports
python3 -c "import sounddevice; import soundfile; import elevenlabs; print('Speaker packages installation successful!')"