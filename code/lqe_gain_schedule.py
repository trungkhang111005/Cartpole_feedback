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
PWM_FREQ = 200
PWM_CHANNEL = 13  # GPIO B (PWM)
DIR_CHANNEL = 6   # GPIO A (Direction)
EN_CHANNEL = 5    # Enable pin
RELAY_CHANNEL = 26  # Relay control
RPM_TO_M_S = 0.071 / 60.0
M_S_TO_RPM = 60.0 / 0.071

K_BASE_THETA = 400
K_MAX_THETA = 2000
THRESH_THETA = 8.0
K_BASE_THETA_DOT = 20
K_MAX_THETA_DOT = 150
THRESH_THETA_DOT = 25.0
K_V = 150
# -------------------------------
# Shared Global Handle
# -------------------------------

gpio_handle = None

# -------------------------------
# IMU Interface
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
	global gpio_handle
	rpm = max(min(rpm, MAX_RPM), -MAX_RPM)
	duty_percent = abs(rpm) / MAX_RPM * 100.0
	direction = 1 if rpm <= 0 else 0
	lgpio.gpio_write(gpio_handle, DIR_CHANNEL, direction)
	try:
		lgpio.tx_pwm(gpio_handle, PWM_CHANNEL, PWM_FREQ, duty_percent)
	except Exception as e:
		print(f"[apply_control ERROR] {e}")

# -------------------------------
# Threads
# -------------------------------

def imu_thread(imu, shared, lock, stop_event):
	while not stop_event.is_set():
		angle, ang_vel = imu.update()
		with lock:
			shared['angle'] = angle
			shared['ang_vel'] = ang_vel
		time.sleep(0.01)

def velocity_thread(shared, lock, stop_event):
	rate_hz = 1000.0
	dt = 1.0 / rate_hz
	tau = 0.01
	alpha = dt / (tau + dt)
	with lock:
		shared['cmd_vel'] = 0.0
		shared['cart_vel'] = 0.0

	next_time = time.perf_counter()
	while not stop_event.is_set():
		with lock:
			cmd = shared['cmd_vel'] * RPM_TO_M_S
			vel = shared['cart_vel']
			vel += alpha * (cmd - vel)
			shared['cart_vel'] = vel
		now = time.perf_counter()
		sleep_time = next_time - now
		if sleep_time > 0:
			time.sleep(sleep_time)
		next_time += dt

def scaled_gain(x, k_base, k_max, x_thresh):
	scale = min(abs(x) / x_thresh, 1.0)
	return k_base + (k_max - k_base) * scale

def control_thread(shared, lock, stop_event):
	rate_hz = 100.0
	dt = 1.0 / rate_hz
	next_time = time.perf_counter()
	while not stop_event.is_set():
		try:
			with lock:
				theta = shared['angle']
				theta_dot = shared['ang_vel']
				v_cart = shared['cart_vel']

			k_theta = scaled_gain(theta, K_BASE_THETA, K_MAX_THETA, THRESH_THETA  )
			k_theta_dot = scaled_gain(theta_dot, K_BASE_THETA_DOT, K_MAX_THETA_DOT, THRESH_THETA_DOT)
			k_v = K_V

			x = [math.radians(theta), math.radians(theta_dot), v_cart]
			u = -(k_theta * x[0] + k_theta_dot * x[1] + k_v * x[2])

			u_clamped = max(min(u, MAX_RPM), -MAX_RPM)

			if (abs(theta) < 1 and abs(theta_dot) < 2.0) or abs(theta) > 30:
				u_clamped = 0.0
			with lock:
				shared['cmd_vel'] = u_clamped
			apply_control(u_clamped)
			now = time.perf_counter()
			sleep_time = next_time - now
			if sleep_time > 0:
				time.sleep(sleep_time)
			next_time += dt
		except Exception as e:
			print(f"[control_thread ERROR] {e}")
			time.sleep(1)

# -------------------------------
# Main
# -------------------------------

def main():
	global gpio_handle
	bus = SMBus(1)
	imu = IMUReader(bus)
	gpio_handle = lgpio.gpiochip_open(4)
	for pin in [DIR_CHANNEL, EN_CHANNEL, RELAY_CHANNEL, PWM_CHANNEL]:
		lgpio.gpio_claim_output(gpio_handle, pin)
	lgpio.gpio_write(gpio_handle, EN_CHANNEL, 0)
	lgpio.gpio_write(gpio_handle, RELAY_CHANNEL, 1)

	shared_data = {'angle': 0.0, 'ang_vel': 0.0, 'cmd_vel': 0.0, 'cart_vel': 0.0}
	lock = threading.Lock()
	stop_event = threading.Event()

	threads = [
		threading.Thread(target=imu_thread, args=(imu, shared_data, lock, stop_event)),
		threading.Thread(target=control_thread, args=(shared_data, lock, stop_event)),
		threading.Thread(target=velocity_thread, args=(shared_data, lock, stop_event))
	]
	for t in threads:
		t.start()

	try:
		while True:
			with lock:
				print(f"θ: {shared_data['angle']:6.2f}°,  θ̇: {shared_data['ang_vel']:6.2f}°/s,  cmd_vel: {shared_data['cmd_vel']:.3f} rpm ({shared_data['cmd_vel']*RPM_TO_M_S:.3f} m/s),  v_cart: {shared_data['cart_vel']:.3f} m/s")
			time.sleep(0.02)
	except KeyboardInterrupt:
		print("[Main] KeyboardInterrupt received. Stopping threads.")
		stop_event.set()
		for t in threads:
			t.join(timeout=2)
	finally:
		try:
			bus.close()
			lgpio.tx_pwm(gpio_handle, PWM_CHANNEL, 0, 0)
			for pin in [EN_CHANNEL, RELAY_CHANNEL, DIR_CHANNEL, PWM_CHANNEL]:
				lgpio.gpio_write(gpio_handle, pin, 0)
			for pin in [DIR_CHANNEL, EN_CHANNEL, RELAY_CHANNEL, PWM_CHANNEL]:
				lgpio.gpio_free(gpio_handle, pin)
			lgpio.gpiochip_close(gpio_handle)
			print("[Shutdown] GPIOs released and motor disabled.")
		except Exception as e:
			print(f"[Shutdown ERROR] {e}")

if __name__ == "__main__":
	main()
