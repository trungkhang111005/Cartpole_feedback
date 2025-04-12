import time
import math
import threading
from smbus2 import SMBus
import lgpio

# ------------------------------------------------------------------
# Constants and Control Gains (tune these for your hardware/system)
# ------------------------------------------------------------------

# ClearPath parameter
TRACK_LEN = 0.85  # Meters
MAX_ROTATIONS = 10.75 # rounds
MAX_RPM = 50 # rounds / sec
# IMU (e.g., MPU6050) constants
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_SEN = 16384.0   # Accelerometer sensitivity
GYRO_SEN = 131.0      # Gyroscope sensitivity (degrees per second per LSB)

# IMU BIAS
BIAS_ACC_X = 0
BIAS_ACC_Y = 170
BIAS_ACC_Z = 1514
BIAS_GYRO_X = -166
BIAS_GYRO_Y = -22
BIAS_GYRO_Z = -140
# Complementary filter weight for IMU fusion
ALPHA_IMU = 0.95

# PD gains for the IMU angle controller
KP_ANGLE = 10.0
KD_ANGLE = 1.5

# PD gains for the servo velocity controller (using HLFB feedback)
KP_VEL = 5
# Target setpoints (upright, zero velocity)
TARGET_ANGLE = 0.0    # degrees
TARGET_VELOCITY = 0.0 # % of max servo velocity

# Loop timing
TARGET_LOOP_TIME = 0.01  # 10 ms

# ------------------------------------------------------------------
# ClearPath Motor Controller and HLFB Reader (from your pwm.py)
# ------------------------------------------------------------------

class ClearPathSD:
    def __init__(self, chip=4, relay_pin=26, enable_pin=5, dir_pin=6, pwm_pin=13):
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


class HLFBReader:
    """
    Reads the HLFB channel PWM from the ClearPath servo. The measured
    duty cycle corresponds to the % of maximum servo velocity.
    """
    def __init__(self, chip=4, pin=19):
        self.chip = lgpio.gpiochip_open(chip)
        self.pin = pin
        lgpio.gpio_claim_input(self.chip, self.pin)

    def measure_duty_cycle(self, timeout=1.0):
        """Measure PWM duty cycle (as % of max velocity) over one period."""
        start_time = time.monotonic()
        timeout_time = start_time + timeout

        # Synchronize: wait for the signal to be low to start at a falling edge
        while lgpio.gpio_read(self.chip, self.pin) == 1:
            if time.monotonic() > timeout_time:
                return None

        # Wait for rising edge: the moment the signal goes high, capture time
        while lgpio.gpio_read(self.chip, self.pin) == 0:
            if time.monotonic() > timeout_time:
                return None
        t_rise = time.monotonic_ns()

        # Wait for falling edge: capture time
        while lgpio.gpio_read(self.chip, self.pin) == 1:
            if time.monotonic() > timeout_time:
                return None
        t_fall = time.monotonic_ns()
        high_time = t_fall - t_rise

        # Wait for next rising edge to get full period
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


# ------------------------------------------------------------------
# Cart Position Compute Through Integration of Feedback Velocity
# ------------------------------------------------------------------
class CartPositionEstimator:
    """
    Estimates the cart position by integrating servo velocity over time.
    Assumes the cart starts at the midpoint of the track.
    Velocity is inferred from HLFB duty cycle as % of MAX_CART_VELOCITY.
    """
    def __init__(self):
        self.position = 0.0  # meters, 0 = center of track
        self.max_velocity = TRACK_LEN * MAX_RPM / MAX_ROTATIONS
        self.track_half_length = TRACK_LEN / 2.0
        self.prev_time = None

    def update(self, duty_cycle_percent: float, direction_forward: bool) -> float:
        """
        Update position estimate based on current velocity reading and time delta.

        Args:
            duty_cycle_percent: HLFB duty cycle as % of max servo velocity.
            direction_forward: True if motor is driving forward, else False.

        Returns:
            Current estimated position in meters.
        """
        current_time = time.time()
        if self.prev_time is None:
            # First call: just set prev_time, don’t integrate
             self.prev_time = current_time
             return self.position
        dt = current_time - self.prev_time
        self.prev_time = current_time
        # Convert HLFB % to real velocity
        velocity = (duty_cycle_percent / 100.0) * self.max_velocity
        signed_velocity = velocity if direction_forward else -velocity

        # Integrate velocity to get position
        self.position += signed_velocity * dt
        # Clamp to physical track limits
        self.position = max(-self.track_half_length, min(self.track_half_length, self.position))
        return self.position

# ------------------------------------------------------------------
# IMU Reader (using MPU6050 over I2C)
# ------------------------------------------------------------------

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

class IMUReader:
    """
    Reads accelerometer and gyroscope data from an MPU6050 sensor,
    and fuses them with a complementary filter to compute the angle.
    """
    def __init__(self, bus, alpha=ALPHA_IMU):
        self.bus = bus
        self.alpha = alpha
        # Wake up MPU6050
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        self.prev_time = time.time()
        # Initial angle estimate from accelerometer (using Y and Z axes)
        ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
        az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
        self.angle = math.degrees(math.atan2(ay, az))

    def update(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Read accelerometer data for angle estimation (Y and Z axes)
        ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 2) - BIAS_ACC_Y
        az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H + 4) - BIAS_ACC_Z
        angle_acc = math.degrees(math.atan2(ay, az))

        # Read gyroscope data (X-axis angular velocity)
        rx = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H) - BIAS_GYRO_X
        ang_vel = rx / GYRO_SEN  # in degrees per second
        angle_gyro = ang_vel * dt
        #print(f"acc_angle: {angle_acc}")
        # Complementary filter to fuse the two measurements
        self.angle = self.alpha * (self.angle + ang_vel * dt) + (1 - self.alpha) * angle_acc
        return self.angle, ang_vel

# ------------------------------------------------------------------
# PD Control Functions
# ------------------------------------------------------------------

def pd_control_angle(current_angle, current_ang_vel, target_angle=TARGET_ANGLE, kp=KP_ANGLE, kd=KD_ANGLE):
    """
    PD controller using IMU-derived angle and angular velocity.
    """
    error = target_angle - current_angle
    control_signal = kp * error - kd * current_ang_vel
    return control_signal

def pd_control_velocity(measured_velocity, target_velocity=TARGET_VELOCITY, kp=KP_VEL):
    """
    PD controller using servo velocity feedback (HLFB measured duty cycle).
    """
    error = target_velocity - measured_velocity
    control_signal = kp * error
    return control_signal

# ------------------------------------------------------------------
# Fusion Control Loop
# ------------------------------------------------------------------

def fusion_control_loop():
    # Set up the motor controller and HLFB reader
    motor = ClearPathSD(chip=4, relay_pin=26, enable_pin=5, dir_pin=6, pwm_pin=13)
    hlfb = HLFBReader(chip=4, pin=19)

    # Set up the IMU on the I2C bus
    bus = SMBus(1)
    imu = IMUReader(bus, alpha=ALPHA_IMU)
    estimator = CartPositionEstimator()
    # Choose control mode:
    # mode 1: use only IMU angle PD control,
    # mode 2: fuse IMU angle PD control with HLFB-based velocity PD control.
    control_mode = 1

    try:
        while True:
            loop_start_time = time.time()
            # Read IMU data
            current_angle, ang_vel = imu.update()

            # Read HLFB feedback (measured % of max servo velocity)
            #measured_velocity = hlfb.measure_duty_cycle()
            #if measured_velocity is None:
             #   # In case of timeout or error reading HLFB, skip this loop iteration.
              #  print("HLFB reading error; skipping iteration.")
               # continue
            if control_mode == 1:
                control_signal = pd_control_angle(current_angle, ang_vel)
            elif control_mode == 2:
                control_signal_angle = pd_control_angle(current_angle, ang_vel)
                control_signal_velocity = pd_control_velocity(measured_velocity)
                # Combine the two control signals. The weighting factors can be tuned.
                control_signal = 0.5 * control_signal_angle + 0.5 * control_signal_velocity * sign(control_signal_angle)
            direction_velocity = control_signal >= 0
            #position = estimator.update(measured_velocity, direction_velocity)
            # Here we map the control_signal to a PWM command (0-100% of max velocity).
            # This mapping will depend on your hardware’s response and desired range.
            pwm_command = max(0, min(95, abs(control_signal)))  # ensure within 0-100%

            # Optionally set motor direction based on the sign of the control signal.
            motor.set_direction(forward=(control_signal >= 0))
            motor.set_speed_percent(pwm_command)

            # Debug printout
            print(f"IMU Angle: {current_angle:6.2f}°, Angular Velocity: {ang_vel:6.2f}°/s, "
                  #f"HLFB Velocity: {measured_velocity:6.2f}%, "
                  f"Control: {control_signal:6.2f}")
                  #f"Estimated Pos: {position:6.2f}meter")

            loop_end_time = time.time()
            elapsed = loop_end_time - loop_start_time
            sleep_time = max(0, TARGET_LOOP_TIME - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Shutting down fusion control loop.")
    finally:
        bus.close()
        hlfb.close()
        motor.close()

if __name__ == "__main__":
    fusion_control_loop()
