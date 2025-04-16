import lgpio
import time
import numpy as np
import matplotlib.pyplot as plt
# ---- CONFIG ----
CHIP = 4
PWM_PIN = 13
EN_PIN = 5
RELAY_PIN = 26
HLFB_PIN = 19
PWM_FREQ = 200

MAX_TORQUE = 0.7  # Nm
APPLIED_TORQUE = 0.25  # Nm
LINEAR_PER_REV = 0.069528  # meters
RADIUS = LINEAR_PER_REV / (2 * np.pi)  # meters
HLFB_FREQ = 45  # Hz → sample every ~22 ms
SAMPLE_DURATION = 1.2  # seconds
ALPHA = 0.99  # smoothing

# ---- SETUP ----
h = lgpio.gpiochip_open(CHIP)
lgpio.gpio_claim_output(h, PWM_PIN)
lgpio.gpio_claim_output(h, EN_PIN)
lgpio.gpio_claim_output(h, RELAY_PIN)
lgpio.gpio_claim_input(h, HLFB_PIN)

# Enable motor
lgpio.gpio_write(h, EN_PIN, 1)
lgpio.gpio_write(h, RELAY_PIN, 1)

# Set torque via PWM
duty = 50 - (APPLIED_TORQUE / MAX_TORQUE) * 50
lgpio.tx_pwm(h, PWM_PIN, PWM_FREQ, duty)
print(f"Applied torque: {APPLIED_TORQUE:.3f} Nm (PWM duty: {duty:.1f}%)")

# ---- HLFB DUTY TO VELOCITY FUNCTION ----
def read_duty_cycle(pin=HLFB_PIN, timeout=1.0):
	start_time = time.monotonic()
	while lgpio.gpio_read(h, pin) == 1:
		if time.monotonic() - start_time > timeout:
			return None
	while lgpio.gpio_read(h, pin) == 0:
		if time.monotonic() - start_time > timeout:
			return None
	t_rise = time.monotonic_ns()
	while lgpio.gpio_read(h, pin) == 1:
		if time.monotonic() - start_time > timeout:
			return None
	t_fall = time.monotonic_ns()
	high_time = t_fall - t_rise
	while lgpio.gpio_read(h, pin) == 0:
		if time.monotonic() - start_time > timeout:
			return None
	t_next_rise = time.monotonic_ns()
	period = t_next_rise - t_rise
	if period == 0:
		return None
	duty = (high_time / period) * 100
	return duty

# ---- DATA COLLECTION ----
velocities = []
timestamps = []
v_filtered = None
v_max = 0.071 * 1500 / 60  # m/s from max RPM

print("Measuring velocity...")
start = time.time()
while time.time() - start < SAMPLE_DURATION:
	duty = read_duty_cycle()
	if duty is not None:
		velocity = (duty / 100.0) * v_max
		v_filtered = ALPHA * velocity + (1 - ALPHA) * (v_filtered or velocity)
		timestamps.append(time.time() - start)
		velocities.append(v_filtered)
	time.sleep(1 / HLFB_FREQ)

# ---- STOP MOTOR ----
lgpio.tx_pwm(h, PWM_PIN, PWM_FREQ, 0)
lgpio.gpio_write(h, EN_PIN, 0)
lgpio.gpio_write(h, RELAY_PIN, 0)
lgpio.gpio_free(h, PWM_PIN)
lgpio.gpio_free(h, EN_PIN)
lgpio.gpio_free(h, RELAY_PIN)
lgpio.gpio_free(h, HLFB_PIN)
lgpio.gpiochip_close(h)

# ---- PROCESS ----
t = np.array(timestamps)
v = np.array(velocities)

cutoff = 0.13
mask = t < cutoff
t_cut = t[mask]
v_cut = v[mask]

# Add origin (0,0) to improve regression realism
t_cut = np.insert(t_cut, 0, 0.0)
v_cut = np.insert(v_cut, 0, 0.0)

if len(t_cut) < 2:
	print("Not enough data collected.")
	exit()

# Linear regression: v ≈ a*t + b
A = np.vstack([t_cut, np.ones_like(t_cut)]).T
a, _ = np.linalg.lstsq(A, v_cut, rcond=None)[0]

if a <= 0:
	print("No measurable acceleration.")
	exit()

# Estimate mass
mass = APPLIED_TORQUE / (RADIUS * a)

print("\n=== MASS ESTIMATION RESULT ===")
print(f"Acceleration: {a:.4f} m/s²")
print(f"Estimated Mass: {mass:.3f} kg")
plt.plot(t, v)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity vs Time")
plt.grid()
plt.savefig("velocity_plot.png")  # Saves to your current directory
print("Velocity plot saved to velocity_plot.png")
