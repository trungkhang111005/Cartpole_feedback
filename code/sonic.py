import lgpio
import time

TRIG = 15    # BCM GPIO 9  (Pin 21)
ECHO = 14  # BCM GPIO 11 (Pin 23)

# Open GPIO chip 0
h = lgpio.gpiochip_open(4)

# Claim TRIG as output and ECHO as input
lgpio.gpio_claim_output(h, TRIG)
lgpio.gpio_claim_input(h, ECHO)
def measure_distance():
	# Send 10 µs trigger pulse
	lgpio.gpio_write(h, TRIG, 0)
	time.sleep(0.000002)  # Ensure low
	lgpio.gpio_write(h, TRIG, 1)
	time.sleep(0.00001)   # 10 µs pulse
	lgpio.gpio_write(h, TRIG, 0)

	# Wait for ECHO to go HIGH
	timeout_start = time.perf_counter()
	while lgpio.gpio_read(h, ECHO) == 0:
		if time.perf_counter() - timeout_start > 0.003:  # 3 ms timeout
			return None
	start_time = time.perf_counter()

	# Wait for ECHO to go LOW
	timeout_start = time.perf_counter()
	while lgpio.gpio_read(h, ECHO) == 1:
		if time.perf_counter() - start_time > 0.005:
			return None
	end_time = time.perf_counter()

	# Duration and distance calculation
	duration = end_time - start_time
	distance = (343.0 * duration) / 2  # meters
	return distance
try:
	while True:
		# Benchmark 100 measurements
		count = 0
		start = time.time()
		last_d = None

		while count < 100:
			d = measure_distance()
			if d:
				last_d = d
				count += 1

		end = time.time()
		avg_time_ms = (end - start) / count * 1000
		print(f"Avg loop time: {avg_time_ms:.2f} ms")
		print(f"Last Distance: {last_d * 100:.2f} cm")

		# Short delay before next benchmark burst
		time.sleep(0.1)

finally:
	lgpio.gpiochip_close(h)
