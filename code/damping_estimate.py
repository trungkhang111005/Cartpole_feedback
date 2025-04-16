import lgpio
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ---- CONFIG ----
CHIP = 4
PWM_PIN = 13
EN_PIN = 5
RELAY_PIN = 26
HLFB_PIN = 19
PWM_FREQ = 200

MAX_TORQUE = 0.7       # Nm (motor rating)
APPLIED_TORQUE = 0.25  # Nm (constant input)
LINEAR_PER_REV = 0.069528  # meters
RADIUS = LINEAR_PER_REV / (2 * np.pi)  # meters
HLFB_FREQ = 45         # Hz
SAMPLE_DURATION = 1.2  # seconds
ALPHA = 0.99           # low-pass filter

# From prior mass estimation
MASS = 4  # kg (replace with measured value)

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

# ---- HLFB READ FUNCTION ----
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
v_max = 0.071 * 1500 / 60  # m/s

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

# ---- DAMPING ESTIMATION ----
t = np.array(timestamps)
v = np.array(velocities)

def velocity_model(t, V_ss, alpha):
	return V_ss * (1 - np.exp(-alpha * t))

# Fit only the first 0.5 seconds
mask = t < 0.5
t_fit = t[mask]
v_fit = v[mask]

if len(t_fit) < 3:
	print("Not enough data for fitting.")
	exit()

params, _ = curve_fit(velocity_model, t_fit, v_fit, p0=[v_fit[-1], 5.0])
V_ss, alpha = params
a0 = APPLIED_TORQUE / (MASS * RADIUS)
b = alpha * MASS

print("\n=== DAMPING ESTIMATION RESULT ===")
print(f"Estimated alpha (b/m): {alpha:.3f} 1/s")
print(f"Estimated V_ss: {V_ss:.3f} m/s")
print(f"Estimated damping b: {b:.4f} N·s/m")

# ---- PLOT ----
plt.plot(t, v, label="Measured")
plt.plot(t, velocity_model(t, V_ss, alpha), '--', label="Fitted Model")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Fit with Damping")
plt.grid()
plt.legend()
plt.savefig("damping_fit_plot.png")
print("Plot saved to damping_fit_plot.png")
