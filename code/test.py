from smbus2 import SMBus
import time
import math

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_SEN = 16384
GYRO_SEN = 131
ALPHA =  0.98
BIAS_ACC_X = 0
BIAS_ACC_Y = 170
BIAS_ACC_Z = 1514
BIAS_GYRO_X = -166
BIAS_GYRO_Y = -22
BIAS_GYRO_Z = -140
#BETA = 0.5
TARGET_LOOP_TIME = 0.01
def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

with SMBus(1) as bus:
    # Wake up MPU6050 (clear sleep bit)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
    prev_time = time.time()
    ang_pos_x = 0
    ay = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2)
    az = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4)
    ang_pos_x_acc_prev = math.degrees(math.atan2(ay,az))
    while True:
        current_time = time.time()
        dt = current_time  - prev_time
        prev_time = current_time
        bias = [0, 170, 1514]
        ax = read_word(bus, MPU_ADDR, ACCEL_XOUT_H) - BIAS_ACC_X
        ay = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
        az = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
        rx = read_word(bus, MPU_ADDR, GYRO_XOUT_H) - BIAS_GYRO_X
        ry = read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2)
        rz = read_word(bus, MPU_ADDR, GYRO_XOUT_H + 4)
        ang_vel_x = rx/GYRO_SEN
        ang_vel_y = ry/GYRO_SEN
        ang_vel_z = rz/GYRO_SEN
        ang_pos_x_acc = math.degrees(math.atan2(ay,az))
        #ang_pos_x_acc = ang_pos_x_acc_prev * BETA + ang_pos_x_acc * (1 - BETA)
        #ang_pox_x_acc_prev = ang_pos_x_acc 
        ang_pos_x = ALPHA * (ang_pos_x + ang_vel_x * dt) + (1 - ALPHA) * ang_pos_x_acc
        #print(f"Accel X: {ax}, Y: {ay}, Z: {az}, Gyro X: {rx}, Y: {ry}, Z: {rz}")
        print(f"Ang_Velocity X: {ang_vel_x}, angle_x: {ang_pos_x}, accelerometer angle x: {ang_pos_x_acc}")
        elapsed_time = time.time() - current_time
        sleep_time = max(0, TARGET_LOOP_TIME - elapsed_time)
        time.sleep(sleep_time)
