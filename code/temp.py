#!/usr/bin/env python3

import subprocess

def read_temp():
	try:
		with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
			temp_milli_c = int(f.read().strip())
			temp_c = temp_milli_c / 1000.0
			print(f"CPU Temperature: {temp_c:.2f}°C")
	except FileNotFoundError:
		print("Temperature file not found. Are you running this on a Raspberry Pi?")
	except Exception as e:
		print(f"Error reading temperature: {e}")

def read_throttled():
	try:
		output = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8").strip()
		print(f"Power Status Raw: {output}")

		if "=" in output:
			_, hex_str = output.split("=")
			flags = int(hex_str, 16)

			if flags == 0:
				print("Power Status: All clear. No throttling or undervoltage detected.")
			else:
				if flags & 0x1:
					print("⚠️ Currently undervolted!")
				if flags & 0x2:
					print("⚠️ Arm frequency capped (current)!")
				if flags & 0x4:
					print("⚠️ Currently throttled!")
				if flags & 0x8:
					print("⚠️ Soft temperature limit active!")

				if flags & 0x10000:
					print("⚠️ Undervoltage occurred since last reboot.")
				if flags & 0x20000:
					print("⚠️ Arm frequency was capped since last reboot.")
				if flags & 0x40000:
					print("⚠️ Throttling occurred since last reboot.")
				if flags & 0x80000:
					print("⚠️ Soft temp limit occurred since last reboot.")
	except FileNotFoundError:
		print("vcgencmd not found. Is the firmware properly installed?")
	except Exception as e:
		print(f"Error reading throttled status: {e}")

if __name__ == "__main__":
	read_temp()
	read_throttled()
