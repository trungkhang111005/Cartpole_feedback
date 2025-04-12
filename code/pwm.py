import threading
import time
import lgpio

# Global motor reference (to allow main to force shutdown)
global_motor = None

# --- Motor Control Class ---
class ClearPathSD:
    def __init__(self, chip=4, relay_pin=5, enable_pin=6, dir_pin=20, pwm_pin=21):
        self.chip = lgpio.gpiochip_open(chip)
        self.relay_pin = relay_pin
        self.enable_pin = enable_pin
        self.dir_pin = dir_pin
        self.pwm_pin = pwm_pin

        # Claim output pins
        for pin in [relay_pin, enable_pin, dir_pin, pwm_pin]:
            lgpio.gpio_claim_output(self.chip, pin, 0)

        # Enable motor power (active low enable)
        self.enable_motor()

    def enable_motor(self):
        lgpio.gpio_write(self.chip, self.relay_pin, 1)
        lgpio.gpio_write(self.chip, self.enable_pin, 0)

    def disable_motor(self):
        # For active low enable, setting the enable pin to 1 disables the motor.
        lgpio.gpio_write(self.chip, self.enable_pin, 1)
        lgpio.gpio_write(self.chip, self.relay_pin, 0)
        lgpio.tx_pwm(self.chip, self.pwm_pin, 0, 0)
        lgpio.gpio_write(self.chip, self.dir_pin, 0)

    def set_direction(self, forward: bool):
        lgpio.gpio_write(self.chip, self.dir_pin, 1 if forward else 0)

    def set_speed_percent(self, percent: float, frequency=50):
        if percent < 0 or percent > 100:
            raise ValueError("Speed percent must be between 0 and 100")
        lgpio.tx_pwm(self.chip, self.pwm_pin, frequency, percent)

    def stop(self):
        self.set_speed_percent(0)

    def close(self):
        self.disable_motor()
        for pin in [self.relay_pin, self.enable_pin, self.dir_pin, self.pwm_pin]:
            lgpio.gpio_free(self.chip, pin)
        lgpio.gpiochip_close(self.chip)
        print("[ClearPathSD] Motor fully disabled.", flush=True)

# --- HLFB Feedback Class ---
class HLFBReader:
    def __init__(self, chip=4, pin=13):
        self.chip = lgpio.gpiochip_open(chip)
        self.pin = pin
        lgpio.gpio_claim_input(self.chip, self.pin)
    def measure_duty_cycle(self, timeout=1.0):
	    """Measure PWM duty cycle from HLFB over a single period."""
	    start_time = time.monotonic()
	    timeout_time = start_time + timeout

	    # Synchronize: wait for the signal to be low to start at a falling edge
	    while lgpio.gpio_read(self.chip, self.pin) == 1:
	        if time.monotonic() > timeout_time:
	            return None

	    # Wait for rising edge: the moment signal goes high, capture t_rise
	    while lgpio.gpio_read(self.chip, self.pin) == 0:
	        if time.monotonic() > timeout_time:
	            return None
	    t_rise = time.monotonic_ns()

	    # Wait for falling edge: capture t_fall immediately
	    while lgpio.gpio_read(self.chip, self.pin) == 1:
	        if time.monotonic() > timeout_time:
	            return None
	    t_fall = time.monotonic_ns()
	    high_time = t_fall - t_rise

	    # Wait for the next rising edge to capture full period
	    while lgpio.gpio_read(self.chip, self.pin) == 0:
	        if time.monotonic() > timeout_time:
	            return None
	    t_rise_next = time.monotonic_ns()
	    period = t_rise_next - t_rise

	    duty = (high_time / period) * 100
	    return round(duty, 2)


    def close(self):
        lgpio.gpio_free(self.chip, self.pin)
        lgpio.gpiochip_close(self.chip)
        print("[HLFBReader] Input released.", flush=True)

# --- Motor Control Thread Function ---
def control_motor(stop_event):
    global global_motor
    motor = ClearPathSD(chip=4, relay_pin=5, enable_pin=6, dir_pin=20, pwm_pin=21)
    global_motor = motor  # Save reference for main-thread forced shutdown
    try:
        print("[Motor] Starting control sequence...", flush=True)
        motor.set_direction(forward=False)
        motor.set_speed_percent(10)
        for i in range(50):  # 5 seconds total, checking every 0.1 sec
          #  print(f"[Motor] Loop 1 step {i}", flush=True)
            if stop_event.is_set():
                print("[Motor] Stop event detected in loop 1", flush=True)
                return
            time.sleep(0.1)
        motor.set_speed_percent(20)
        for i in range(50):  # Another 5 seconds
            #print(f"[Motor] Loop 2 step {i}", flush=True)
            if stop_event.is_set():
                print("[Motor] Stop event detected in loop 2", flush=True)
                return
            time.sleep(0.1)
        motor.stop()
        print("[Motor] Sequence complete.", flush=True)
    except Exception as e:
        print(f"[Motor] Error: {e}", flush=True)
    finally:
        print("[Motor] Entering finally block; cleaning up motor.", flush=True)
        try:
            motor.close()
        except Exception as ex:
            print(f"[Motor] Exception during motor.close(): {ex}", flush=True)
        print("[Motor] Shutdown complete.", flush=True)

# --- HLFB Monitor Thread Function ---
def monitor_hlfb(stop_event):
    hlfb = HLFBReader(chip=4, pin=13)
    try:
        while not stop_event.is_set():
            duty = hlfb.measure_duty_cycle()
            if duty is not None:
                print(f"[HLFB] Duty cycle: {duty:.2f}%", flush=True)
            else:
                print("[HLFB] No signal detected.", flush=True)
            time.sleep(0.5)
    except Exception as e:
        print(f"[HLFB] Error: {e}", flush=True)
    finally:
        print("[HLFB] Entering finally block; cleaning up HLFB.", flush=True)
        try:
            hlfb.close()
        except Exception as ex:
            print(f"[HLFB] Exception during hlfb.close(): {ex}", flush=True)
        print("[HLFB] Shutdown complete.", flush=True)

# --- Main Program ---
if __name__ == "__main__":
    stop_event = threading.Event()
    # For positive logic: threads run while the event is set
    stop_event.clear()
    motor_thread = threading.Thread(target=control_motor, args=(stop_event,))
    hlfb_thread = threading.Thread(target=monitor_hlfb, args=(stop_event,))

    motor_thread.start()
    hlfb_thread.start()

    try:
        # Wait for the motor thread to finish its sequence
        motor_thread.join()
        print("[Main] Motor thread finished. Press Ctrl+C to stop HLFB monitor.", flush=True)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[Main] KeyboardInterrupt received. Signaling stop_event.", flush=True)
        stop_event.set()  # Signal threads to stop
        motor_thread.join(timeout=5)
        hlfb_thread.join(timeout=5)
       # if global_motor is not None:
        #    try:
         #       print("[Main] Calling global_motor.close() to force shutdown.", flush=True)
          #      global_motor.close()
           # except Exception as e:
            #    print(f"[Main] Error while force closing global_motor: {e}", flush=True)
        #print("[Main] Clean shutdown.", flush=True)
