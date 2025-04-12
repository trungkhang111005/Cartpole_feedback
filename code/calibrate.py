#!/usr/bin/env python3
"""
MPU6050 Accelerometer Bias Calibration Script
Author: ChatGPT
Date: 2025-03-24

Place the sensor with each axis (X, Y, Z) pointing upward one at a time.
A 5-second countdown precedes each measurement phase.
"""

from smbus2 import SMBus
import time
import numpy as np

# ------------------ MPU6050 CONFIGURATION ------------------
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
PWR_MGMT_1   = 0x6B

# Initialize I2C bus (1 for modern Raspberry Pi, 0 for older models)
bus = SMBus(1)

def mpu6050_init():
    # Wake up MPU6050 from sleep
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_accel_vector():
    ax = read_raw_data(ACCEL_XOUT_H)
    ay = read_raw_data(ACCEL_XOUT_H + 2)
    az = read_raw_data(ACCEL_XOUT_H + 4)
    return np.array([ax, ay, az], dtype=np.float32)

def calibrate_accel_bias(samples=1000, delay=0.002):
    axes = ['X-axis (pointing UP)', 'Y-axis (pointing UP)', 'Z-axis (pointing UP)']
    expected_vectors = [
        np.array([16384.0, 0.0, 0.0]),  # X-up
        np.array([0.0, 16384.0, 0.0]),  # Y-up
        np.array([0.0, 0.0, 16384.0])   # Z-up
    ]

    all_biases = []

    for i in range(3):
        print(f"\n👉 Please place the sensor with the {axes[i]}")
        print("Calibration will start in 5 seconds...")
        for t in range(5, 0, -1):
            print(f"  {t}...", end='\r')
            time.sleep(1)

        print("📡 Collecting samples...")

        acc_samples = []
        for _ in range(samples):
            acc = get_accel_vector()
            acc_samples.append(acc)
            time.sleep(delay)

        acc_samples = np.array(acc_samples)
        acc_avg = np.mean(acc_samples, axis=0)
        bias = acc_avg - expected_vectors[i]
        all_biases.append(bias)

        print(f"✔️  Measured average: {acc_avg}")
        print(f"🧮 Bias for {axes[i]}: {bias}")

    combined_bias = np.mean(all_biases, axis=0)
    print("\n✅ Final estimated accelerometer bias (x, y, z):", combined_bias)
    return combined_bias
# ------------------ Gyro Read ------------------
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_gyro_vector():
    gx = read_raw_data(GYRO_XOUT_H)
    gy = read_raw_data(GYRO_XOUT_H + 2)
    gz = read_raw_data(GYRO_XOUT_H + 4)
    return [gx, gy, gz]

# ------------------ Gyro Bias Calibration ------------------
def calibrate_gyro_bias(samples=5000, delay=0.002):
    print("📦 Ensure the MPU6050 is completely still")
    print("Calibration will start in 5 seconds...")

    for t in range(5, 0, -1):
        print(f"  {t}...", end="\r")
        time.sleep(1)

    print("📡 Collecting gyroscope samples...")

    sum_gyro = [0.0, 0.0, 0.0]
    for _ in range(samples):
        gx, gy, gz = get_gyro_vector()
        sum_gyro[0] += gx
        sum_gyro[1] += gy
        sum_gyro[2] += gz
        time.sleep(delay)

    bias = [s / samples for s in sum_gyro]
    print("\n✅ Gyroscope Bias (raw LSB values):")
    print(f"    X: {bias[0]:.3f}")
    print(f"    Y: {bias[1]:.3f}")
    print(f"    Z: {bias[2]:.3f}")
    return bias

# ------------------ Main ------------------
if __name__ == "__main__":
    try:
        mpu6050_init()
        bias = calibrate_gyro_bias()
    except KeyboardInterrupt:
        print("\n⛔ Interrupted by user.")
    except Exception as e:
        print("❌ Unexpected error:", e)
