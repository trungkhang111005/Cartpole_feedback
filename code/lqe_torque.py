import time
import math
import threading
from smbus2 import SMBus
import lgpio

# -------------------------------
# Constants and Parameters
# -------------------------------

# IMU constants
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_SEN = 16384.0
GYRO_SEN = 131.0

BIAS_ACC_Y = 170
BIAS_ACC_Z = 1514
BIAS_GYRO_X = -166
ALPHA_THETA = 0.98
ALPHA_THETA_DOT = 0.09
# Servo and physical constants
MAX_TORQUE = 1.4  # Nm
GEAR_RADIUS = 0.071 / (2 * math.pi)  # meters
M_CART = 0.8  # kg
TAU = 0.001  # unused here but kept for tuning
PWM_FREQ = 200
PWM_CHANNEL = 13
EN_CHANNEL = 5
RELAY_CHANNEL = 26
A_CHANNEL = 6
# Gain scheduling constants
K_BASE_THETA = 5.8392
K_MAX_THETA = 5.8392
THRESH_THETA = 8
K_BASE_THETA_DOT = 1.4885
K_MAX_THETA_DOT = 1.4885
THRESH_THETA_DOT = 25.0
K_V = -0.4472

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
		self.alpha_theta = ALPHA_THETA
		self.alpha_theta_dot = ALPHA_THETA_DOT
		self.prev_time = time.perf_counter()
		ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
		az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
		self.angle = math.degrees(math.atan2(ay, az))
		self.angle_vel = 0.0
	def update(self):
		now = time.perf_counter()
		dt = now - self.prev_time
		self.prev_time = now
		ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
		az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
		angle_acc = math.degrees(math.atan2(ay, az))
		rx = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H) - BIAS_GYRO_X
		gyro_measurement = rx / GYRO_SEN
		self.angle_vel = self.alpha_theta_dot * self.angle_vel + (1 - self.alpha_theta_dot) * gyro_measurement
		self.angle = self.alpha_theta * (self.angle + self.angle_vel * dt) + (1 - self.alpha_theta) * angle_acc
		return self.angle, self.angle_vel
	def stop_sensor(self):
		try:
			self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x40)  # Send sleep signal
			print("[IMU] Sleep mode activated.")
		except OSError as e:
			print(f"[IMU STOP ERROR] Could not send stop signal: {e}")

# -------------------------------
# Control Signal Application
# -------------------------------

def apply_torque_control(torque):
	global gpio_handle
	torque = max(min(torque, MAX_TORQUE), -MAX_TORQUE)
	duty_percent = 50.0 + (torque / MAX_TORQUE) * 50.0
	duty_percent = max(min(duty_percent, 100.0), 0.0)
	try:
		lgpio.tx_pwm(gpio_handle, PWM_CHANNEL, PWM_FREQ, duty_percent)
	except Exception as e:
		print(f"[apply_torque_control ERROR] {e}")

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

def scaled_gain(x, k_base, k_max, x_thresh):
	scale = min(abs(x) / x_thresh, 1.0)
	return k_base + (k_max - k_base) * scale

def control_thread(shared, lock, stop_event):
	global gpio_handle
	rate_hz = 100.0
	dt = 1.0 / rate_hz
	next_time = time.perf_counter()
	# Add before while loop
	prev_torque = 0.0
	max_delta = 0.05  # max change per step in Nm
	last_CHA_state = None
	while not stop_event.is_set():
		try:
			with lock:
				theta = shared['angle']
				theta_dot = shared['ang_vel']

			k_theta = scaled_gain(theta, K_BASE_THETA, K_MAX_THETA, THRESH_THETA)
			k_theta_dot = scaled_gain(theta_dot, K_BASE_THETA_DOT, K_MAX_THETA_DOT, THRESH_THETA_DOT)

			x = [math.radians(theta), math.radians(theta_dot), 0.0]

			#if (abs(theta) < 1 and abs(theta_dot) < 2.0) or abs(theta) > 30:
			#	torque = 0.0
			if abs(theta) > 10:
				CHA_state = 0
				torque = 0.0
			else:
				CHA_state = 1
				torque = -(k_theta * x[0] + k_theta_dot * x[1] + K_V * x[2])
			if CHA_state != last_CHA_state:
				lgpio.gpio_write(gpio_handle, A_CHANNEL, CHA_state)
				last_CHA_state = CHA_state
			# Inside loop, before applying torque
			delta = torque - prev_torque
			if abs(delta) > max_delta:
				torque = prev_torque + max_delta * math.copysign(1, delta)
			prev_torque = torque
			with lock:
                                shared['torque'] = torque
			apply_torque_control(torque)

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
	try:
		imu = IMUReader(bus)
	except RuntimeError as e:
		print(f"[Main ERROR] {e}")
		bus.close()
		return  # Exit gracefully
	gpio_handle = lgpio.gpiochip_open(4)
	for pin in [EN_CHANNEL, RELAY_CHANNEL, PWM_CHANNEL, A_CHANNEL]:
		lgpio.gpio_claim_output(gpio_handle, pin)
	lgpio.gpio_write(gpio_handle, EN_CHANNEL, 0)
	lgpio.gpio_write(gpio_handle, RELAY_CHANNEL, 1)
	lgpio.gpio_write(gpio_handle, A_CHANNEL, 0)
	shared_data = {'angle': 0.0, 'ang_vel': 0.0, 'torque' :0.0}
	lock = threading.Lock()
	stop_event = threading.Event()

	threads = [
		threading.Thread(target=imu_thread, args=(imu, shared_data, lock, stop_event)),
		threading.Thread(target=control_thread, args=(shared_data, lock, stop_event))
	]
	for t in threads:
		t.start()

	try:
		while True:
			with lock:
				print(f"θ: {shared_data['angle']:6.2f}°,  θ̇: {shared_data['ang_vel']:6.2f}°/s, torque: {shared_data['torque']:6.2f}Nm ")
			time.sleep(0.02)
	except KeyboardInterrupt:
		print("[Main] KeyboardInterrupt received. Stopping threads.")
		stop_event.set()
		for t in threads:
			t.join(timeout=2)
	finally:
		try:
			imu.stop_sensor()
			bus.close()
			lgpio.tx_pwm(gpio_handle, PWM_CHANNEL, PWM_FREQ, 0)
			for pin in [EN_CHANNEL, RELAY_CHANNEL, PWM_CHANNEL, A_CHANNEL]:
				lgpio.gpio_write(gpio_handle, pin, 0)
				lgpio.gpio_free(gpio_handle, pin)
			lgpio.gpiochip_close(gpio_handle)
			print("[Shutdown] GPIOs released and motor disabled.")
		except Exception as e:
			print(f"[Shutdown ERROR] {e}")

if __name__ == "__main__":
	main()

