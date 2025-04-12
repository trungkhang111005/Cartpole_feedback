
import time
import math
import threading
from smbus2 import SMBus
import lgpio

# -------------------------------
# Constants
# -------------------------------

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_SEN = 16384.0
GYRO_SEN = 131.0

BIAS_ACC_Y = 170
BIAS_ACC_Z = 1514
BIAS_GYRO_X = -166
ALPHA_IMU = 0.95

TRIG = 9   # GPIO BCM
ECHO = 11  # GPIO BCM

# -------------------------------
# Sensor Classes
# -------------------------------

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    return value - 65536 if value > 32767 else value

class IMUReader:
    def __init__(self, bus):
        self.bus = bus
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        self.alpha = ALPHA_IMU
        self.prev_time = time.perf_counter()
        ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
        az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
        self.angle = math.degrees(math.atan2(ay, az))

    def update(self):
        now = time.perf_counter()
        dt = now - self.prev_time
        self.prev_time = now

        ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
        az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
        angle_acc = math.degrees(math.atan2(ay, az))

        rx = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H) - BIAS_GYRO_X
        ang_vel = rx / GYRO_SEN

        self.angle = self.alpha * (self.angle + ang_vel * dt) + (1 - self.alpha) * angle_acc
        return self.angle, ang_vel

class UltrasonicSensor:
    def __init__(self, chip=4, trig=TRIG, echo=ECHO):
        self.chip = lgpio.gpiochip_open(chip)
        self.trig = trig
        self.echo = echo
        lgpio.gpio_claim_output(self.chip, trig)
        lgpio.gpio_claim_input(self.chip, echo)

    def measure_distance(self):
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(0.000002)
        lgpio.gpio_write(self.chip, self.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig, 0)

        timeout = 0.03
        start = time.perf_counter()
        while lgpio.gpio_read(self.chip, self.echo) == 0:
            if time.perf_counter() - start > timeout:
                return -1
        pulse_start = time.perf_counter()

        while lgpio.gpio_read(self.chip, self.echo) == 1:
            if time.perf_counter() - pulse_start > timeout:
                return -1
        pulse_end = time.perf_counter()

        pulse_duration = pulse_end - pulse_start
        distance = (343.0 * pulse_duration) / 2
        return distance

    def close(self):
        lgpio.gpiochip_close(self.chip)

# -------------------------------
# Thread Functions
# -------------------------------

def imu_thread(imu, shared_data, lock):
    while True:
        angle, ang_vel = imu.update()
        with lock:
            shared_data['angle'] = angle
            shared_data['ang_vel'] = ang_vel
        time.sleep(0.01)  # ~100 Hz

def ultrasonic_thread(sonic, shared_data, lock):
	alpha = 0.85  # EMA coefficient (higher = smoother, lower = more responsive)
	prev_dist = None
	prev_time = time.perf_counter()
	vel_est = 0.0  # initial velocity estimate

	while True:
		distance = sonic.measure_distance()
		now = time.perf_counter()

		if prev_dist is not None:
			dt = now - prev_time
			if dt > 0.001:  # protect against zero-division
				raw_vel = (distance - prev_dist) / dt
				vel_est = alpha * vel_est + (1 - alpha) * raw_vel

		with lock:
			shared_data['distance'] = distance
			shared_data['cart_vel'] = vel_est

		prev_dist = distance
		prev_time = now
		time.sleep(0.05)  # ~20 Hz

def control_thread(shared_data, lock):
	# Example LQR gain vector (you should tune this)
	K = [-59.0533516897844, -11.5266991029617, -10.0, -12.4190276752213]  # [k_theta, k_theta_dot, k_position]
	control_rate = 100.0  # Hz
	dt = 1.0 / control_rate
	next_time = time.perf_counter()

	while True:
		with lock:
			theta = shared_data['angle']
			theta_dot = shared_data['ang_vel']
			x = shared_data['distance']
			v = shared_data['cart_vel']
		# State vector
		x_vec = [theta, theta_dot, x, v]

		# Control law: u = -Kx
		u = -sum(k * xi for k, xi in zip(K, x_vec))

		# Apply control
		apply_control(u)

		# Maintain timing
		now = time.perf_counter()
		sleep_time = next_time - now
		if sleep_time > 0:
			time.sleep(sleep_time)
		next_time += dt

# -------------------------------
# Main Thread
# -------------------------------

def main():
    bus = SMBus(1)
    imu = IMUReader(bus)
    sonic = UltrasonicSensor()

    shared_data = {'angle': 0.0, 'ang_vel': 0.0, 'distance': 0.0, 'cart_vel' :0.0}
    lock = threading.Lock()

    imu_t = threading.Thread(target=imu_thread, args=(imu, shared_data, lock), daemon=True)
    sonic_t = threading.Thread(target=ultrasonic_thread, args=(sonic, shared_data, lock), daemon=True)
    ctrl_t = threading.Thread(target=control_thread, args=(shared_data, lock), daemon=True)
    ctrl_t.start()
    imu_t.start()
    sonic_t.start()

    try:
        while True:
            with lock:
                angle = shared_data['angle']
                ang_vel = shared_data['ang_vel']
                distance = shared_data['distance']
                cart_vel = shared_data['cart_vel']
            print(f"θ: {angle:6.2f}°,  θ̇: {ang_vel:6.2f}°/s,  x: {distance:.3f} m,  v: {cart_vel:.3f} m/s")
            time.sleep(0.02)  # UI print rate ~50 Hz
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        bus.close()
        sonic.close()

if __name__ == "__main__":
    main()
