import time
import math
import threading
from smbus2 import SMBus
import lgpio

# -------------------------------
# Constants and Parameters
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

TAU = 0.001
MAX_RPM = 700.0
PWM_FREQ = 20000  # 20 kHz
PWM_CHIP = 0
PWM_CHANNEL = 13  # GPIO B (PWM input to servo)
DIR_CHANNEL = 6  # GPIO A (Direction input)
EN_CHANNEL = 5   # Enable pin (optional)
RELAY_CHANNEL = 26  # Relay enable pin (optional)

# -------------------------------
# Sensor Interfaces
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

# -------------------------------
# Control Signal Application
# -------------------------------

def apply_control(rpm):
	handle = lgpio.gpiochip_open(4)
	lgpio.gpio_claim_output(handle, DIR_CHANNEL)
	lgpio.gpio_claim_output(handle, EN_CHANNEL)
	lgpio.gpio_claim_output(handle, RELAY_CHANNEL)

	# Enable servo power and relay
	lgpio.gpio_write(handle, EN_CHANNEL, 0)
	lgpio.gpio_write(handle, RELAY_CHANNEL, 1)

	rpm = max(min(rpm, MAX_RPM), -MAX_RPM)
	duty = abs(rpm) / MAX_RPM
	period_ns = int(1e9 / PWM_FREQ)
	pulse_ns = int(duty * period_ns)

	# Direction: 1 = forward, 0 = reverse
	direction = 1 if rpm >= 0 else 0
	lgpio.gpio_write(handle, DIR_CHANNEL, direction)

	lgpio.tx_pwm(handle, PWM_CHANNEL, period_ns, pulse_ns)

# -------------------------------
# (Other functions unchanged, insert updated control logic here as needed)
# -------------------------------
def imu_thread(imu, shared, lock):
	while True:
		angle, ang_vel = imu.update()
		with lock:
			shared['angle'] = angle
			shared['ang_vel'] = ang_vel
		time.sleep(0.01)  # ~100 Hz sampling rate
# -------------------------------
# Velocity Estimation Thread
# -------------------------------

def velocity_thread(shared, lock):
	# We model first-order response to commanded velocity
	rate_hz = 1000.0
	dt = 1.0 / rate_hz
	next_time = time.perf_counter()

	with lock:
		shared['cmd_vel'] = 0.0
		shared['cart_vel'] = 0.0

	while True:
		with lock:
			cmd = shared['cmd_vel']
			vel = shared['cart_vel']
			vel += (cmd - vel) * (dt / TAU)
			shared['cart_vel'] = vel

		now = time.perf_counter()
		sleep_time = next_time - now
		if sleep_time > 0:
			time.sleep(sleep_time)
		next_time += dt

# -------------------------------
# Control Law Thread
# -------------------------------
def control_thread(shared, lock):
	K = [41.7400, 10.4789, 77.3125]
	rate_hz = 100.0
	dt = 1.0 / rate_hz
	next_time = time.perf_counter()

	while True:
		try:
			with lock:
				theta = shared['angle']
				theta_dot = shared['ang_vel']
				v_cart = shared['cart_vel']
			x = [math.radians(theta), math.radians(theta_dot), v_cart]
			u = -sum(k * xi for k, xi in zip(K, x))
			with lock:
				shared['cmd_vel'] = u

			print(f"[control] θ: {theta:.2f}, θ̇: {theta_dot:.2f}, v: {v_cart:.3f}, cmd_vel: {u:.2f}")

			now = time.perf_counter()
			sleep_time = next_time - now
			if sleep_time > 0:
				time.sleep(sleep_time)
			next_time += dt

		except Exception as e:
			print(f"[control_thread ERROR] {e}")
			time.sleep(1)

# -------------------------------
# Main Execution
# -------------------------------

def main():
	bus = SMBus(1)
	imu = IMUReader(bus)

	shared_data = {'angle': 0.0, 'ang_vel': 0.0, 'cmd_vel': 0.0, 'cart_vel': 0.0}
	lock = threading.Lock()

	threads = [
		threading.Thread(target=imu_thread, args=(imu, shared_data, lock), daemon=True),
		threading.Thread(target=control_thread, args=(shared_data, lock), daemon=True),
		threading.Thread(target=velocity_thread, args=(shared_data, lock), daemon=True)
	]
	for t in threads: t.start()

	try:
		while True:
			with lock:
				print(f"θ: {shared_data['angle']:6.2f}°,  θ̇: {shared_data['ang_vel']:6.2f}°/s,  v_cart: {shared_data['cart_vel']:.3f} m/s")
			time.sleep(0.02)
	except KeyboardInterrupt:
		print("Terminating...")
	finally:
		bus.close()

if __name__ == "__main__":
	main()
