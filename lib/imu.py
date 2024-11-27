# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from lib.madgwickahrs import MadgwickAHRS, Quaternion

import time
import board
import adafruit_mpu6050

class FilteredMPU6050():
    
    def __init__(self):
        self.sensor = adafruit_mpu6050.MPU6050(board.I2C())
        # self.sensor.gyro_range = adafruit_mpu6050.GyroRange.RANGE_500_DPS  # Set gyroscope range to ±1000 dps
        self.ahrs = MadgwickAHRS(beta=0.008, zeta=0.)
        self.alpha = 1.0  # LPF alpha: x[t] := a*x[t] + (1-a)*x[t-1]
        self.gyro_bias = np.array([0., 0., 0.])

    def calibrate(self):
        try:
            self.gyro_bias = np.loadtxt('gyro_bias.txt')
        except FileNotFoundError:
            self.gyro_bias = np.array([0., 0., 0.])
            for _ in range(50):
                _, gyro = self.read_sensor()
                self.gyro_bias += gyro / 50
                time.sleep(0.01)
            print('Calculated gyro bias:', self.gyro_bias)
            np.savetxt('gyro_bias.txt', self.gyro_bias)

        self.accel, gyro_raw = self.read_sensor()
        self.gyro = gyro_raw - self.gyro_bias
        self.t = time.time()

        self.quat = self._calculate_initial_q(self.accel)
        self.grav = self.quat_rotate(self.quat.conj(), [0, 0, 1])
        self.ahrs.quaternion = self.quat


    def get_orientation(self):
        self.update()
        gx, gy, gz = self.grav

        # Map gravity vector components to new axes
        gX_new = gy       # gX_new = gy
        gY_new = -gx      # gY_new = -gx
        gZ_new = gz       # gZ_new = gz

        # Compute roll and pitch using the standard formulas
        roll = np.degrees(np.arctan2(gY_new, gZ_new))
        pitch = np.degrees(np.arctan2(-gX_new, np.sqrt(gY_new**2 + gZ_new**2)))

        # Compute yaw from the quaternion
        qw, qx, qy, qz = self.quat
        yaw = np.degrees(np.arctan2(2 * (qw * qz + qx * qy),
                                    1 - 2 * (qy**2 + qz**2)))

        return pitch, roll, yaw

    def _calculate_initial_q(self, accel):
        acc_norm = accel / np.linalg.norm(accel)

        # Estimate initial roll and pitch from accelerometer
        initial_roll = np.arctan2(acc_norm[1], acc_norm[2])
        initial_pitch = np.arctan2(-acc_norm[0], np.sqrt(acc_norm[1]**2 + acc_norm[2]**2))
        initial_yaw = 0

        # Initialize quaternion using the from_angle_axis function
        initial_q = Quaternion.from_angle_axis(initial_roll, 1, 0, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_pitch, 0, 1, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_yaw, 0, 0, 1)
        return initial_q

    def read_sensor(self):
        # Read raw data from sensor
        accel = np.array(self.sensor.acceleration)
        gyro = np.array(self.sensor.gyro)
        accel_mapped = np.array([-accel[1], accel[0], accel[2]])
        gyro_mapped = np.array([-gyro[1], gyro[0], gyro[2]])
        return accel_mapped, gyro_mapped


    def update(self):
        # Read and map sensor readings
        self.accel, gyro_raw = self.read_sensor()
        self.gyro = gyro_raw - self.gyro_bias
        t = time.time()

        # Store raw data
        self.accel_RAW = self.accel
        self.gyro_RAW = self.gyro
        self.quat_RAW = self._calculate_initial_q(self.accel_RAW)
        self.grav_RAW = self.quat_rotate(self.quat_RAW.conj(), [0, 0, 1])

        # Filtering
        self.ahrs.samplePeriod = t - self.t
        self.ahrs.update_imu(self.gyro, self.accel)
        self.t = t

        # Update orientation
        quat = self.ahrs.quaternion
        self.quat = quat.q
        self.grav = self.quat_rotate(quat.conj(), [0, 0, 1])


    def quat_rotate(self, q, v):
        """Rotate a vector v by a quaternion q"""
        qv = np.concatenate(([0], v))
        return (q * Quaternion(qv) * q.conj()).q[1:]
    
if __name__ == '__main__':
    imu = FilteredMPU6050()
    imu.calibrate()
    start_time = time.time()
    times = []
    quats = []
    accels = []
    gyros = []
    gyros_raw = []
    gravs = []
    pitches = []
    while time.time() < start_time + 10:
        # pitch = imu.robot_angle()
        pitch, roll, yaw = imu.get_orientation()
        t = time.time()
        pitches.append(pitch)
        times.append(t)
        quats.append(imu.quat)
        accels.append(imu.accel)
        gyros.append(imu.gyro)
        gyros_raw.append(imu.gyro_RAW)
        gravs.append(imu.grav)

        # Print current readings
        # quat = Quaternion(imu.quat)
        # euler = quat.to_euler_angles()
        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
        # print(f"Accel X: {imu.accel[0]:.2f}, Y: {imu.accel[1]:.2f}, Z: {imu.accel[2]:.2f} m/s²")
        print(f"Gyro (filtered) X: {imu.gyro[0]:.2f}, Y: {imu.gyro[1]:.2f}, Z: {imu.gyro[2]:.2f} rad/s")
        # print(f"Gyro (raw) X: {imu.gyro_RAW[0]:.2f}, Y: {imu.gyro_RAW[1]:.2f}, Z: {imu.gyro_RAW[2]:.2f} rad/s")
        print("---")

    times = np.array(times) - times[0]
    print('Average Hz:', 1 / (np.mean(times[1:] - times[:-1])))

    plt.figure()
    gyros = np.stack(gyros)
    gyros_raw = np.stack(gyros_raw)
    sx, sy, sz = np.std(gyros, axis=0)
    plt.title(f'Gyro, std=({sx:.4f}, {sy:.4f}, {sz:.4f})')
    plt.plot(times, gyros, label='filtered')
    plt.plot(times, gyros_raw, '--', alpha=0.4, label='raw')
    plt.legend()
    plt.savefig('gyro.png')

    plt.figure()
    accels = np.stack(accels)
    sx, sy, sz = np.std(accels, axis=0)
    plt.title(f'Accel, std=({sx:.4f}, {sy:.4f}, {sz:.4f})')
    plt.plot(times, accels)
    plt.savefig('accel.png')

    def grav2pitch(gravs):
        return np.degrees(np.atan2(gravs[..., 2], np.sqrt(gravs[..., 0]**2 + gravs[..., 1]**2)))

    plt.figure()
    plt.plot(times, grav2pitch(accels), label='noisy', alpha=0.4)
    plt.plot(times, pitches, label='filtered')
    plt.legend()
    plt.savefig('pitch.png')