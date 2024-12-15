import numpy as np
import matplotlib.pyplot as plt

# Load pitch data from files
pitch_data = np.loadtxt('pitch.debug')
time, pitch = pitch_data.T
raw_pitch = np.loadtxt('raw_pitch.debug')[:,1]
time = np.arange(len(pitch))

# Plot both raw and regular pitch data
plt.figure(figsize=(12,6))
plt.plot(time, raw_pitch, label='Raw Pitch')
plt.plot(time, pitch, label='Filtered Pitch')
plt.title('Pitch Data Comparison')
plt.xlabel('Sample Time')
plt.ylabel('Pitch (degrees)')
plt.grid(True)
plt.legend()
plt.savefig('pitch_comparison.png')
