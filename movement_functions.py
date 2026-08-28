from pybricks.hubs import *
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import *
from umath import sqrt

ev3 = EV3Brick()

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
claw = Motor(Port.C, Direction.COUNTERCLOCKWISE)
arm = Motor(Port.D, Direction.COUNTERCLOCKWISE)
gyro = GyroSensor(Port.S4)
sensorL = ColorSensor(Port.S3)
sensorR = ColorSensor(Port.S2)

#default settings: speed-243, straightacc-141, turnspeed-161, turnacc-908
#not tweaked
db = DriveBase(
    left_motor=left_motor,
    right_motor=right_motor,
    wheel_diameter=93.74,
    axle_track=144
)

# ---- CALIBRATE THIS ----
# Run calibrate_gyro() once, physically mark start/end heading with tape,
# then set GYRO_SCALE = (degrees you actually measured) / (gyro reading printed).
# EV3 gyros commonly under-report by a few percent -> this is likely your
# real "underturning" root cause, not just the control loop.
GYRO_SCALE = 1.037
DRIFT_RATE = 0.0

gyro_watch = StopWatch()
integral = 0

def start_gyro_tracking():
    gyro.reset_angle(0)
    gyro_watch.reset()

def gyro_angle():
    elapsed_s = gyro_watch.time() / 1000
    corrected = gyro.angle() - DRIFT_RATE * elapsed_s
    return corrected * GYRO_SCALE

def calibrate_gyro(turns=3, speed=150):
    """Run this ONCE by itself. Physically mark the robot's heading with
    tape before running. It spins `turns` full rotations, then prints the
    gyro's reported angle. Measure the ACTUAL angle turned with a protractor
    (or count exact wheel marks) and set:
        GYRO_SCALE = actual_degrees / printed_reading
    """
    start_gyro_tracking()
    target = 360 * turns
    while gyro_angle() < target:
        db.drive(0, speed)
        wait(10)
    db.hold()
    wait(300)
    ev3.screen.clear()
    ev3.screen.print("gyro reports:")
    ev3.screen.print(gyro_angle())
    print("gyro reports:", gyro_angle(), "expected", target)

def turn_gyro(target_angle, max_speed=250, decel=400, min_speed=40, tolerance=1, settle_speed=25, timeout_ms=4000):
    """Two-stage turn: fast sqrt-profile approach, then a slow creep 'settle'
    stage that only exits once the angle has genuinely held inside tolerance
    for a few consecutive reads (kills the old momentum/stiction undershoot).
    """
    start_gyro_tracking()

    # --- Stage 1: fast approach, floored on BOTH accel and decel sides ---
    while True:
        current = gyro_angle()
        error = target_angle - current
        if abs(error) <= tolerance:
            break
        if gyro_watch.time() > timeout_ms:
            break  # safety: never hang the run if something is jammed
        traveled = abs(current)
        remaining = abs(error)
        speed_ramp_up = sqrt(2 * decel * traveled) + min_speed
        speed_ramp_down = sqrt(2 * decel * remaining) + min_speed  # floor added here too
        speed = min(speed_ramp_up, speed_ramp_down, max_speed)
        if error < 0:
            speed = -speed
        db.drive(0, speed)
        wait(10)

    db.hold()   # hard brake immediately, no coast — coasting overshoots
                # further in the turn direction every time, which is exactly
                # the kind of same-direction, per-turn bias you're seeing
    wait(150)   # let the arm/momentum settle before trusting the reading

    # --- Stage 2: low-speed settle/creep, requires angle to HOLD before exiting ---
    stable_count = 0
    settle_start = StopWatch()
    while stable_count < 3:
        current = gyro_angle()
        error = target_angle - current
        if abs(error) <= tolerance:
            stable_count += 1
            db.hold()
            wait(20)
            continue
        if settle_start.time() > 1500:
            break  # safety timeout on the settle stage too
        stable_count = 0
        speed = settle_speed if error > 0 else -settle_speed
        db.drive(0, speed)
        wait(10)

    db.hold()

def straight_gyro(distance_mm, speed=200, kp=0.5, ki=0.05):
    global integral
    db.reset()
    start_gyro_tracking()
    integral = 0
    signed_speed = speed if distance_mm >= 0 else -speed
    while abs(db.distance()) < abs(distance_mm):
        angle = gyro_angle()
        integral += angle * 0.01
        turn_rate = -(angle * kp + integral * ki)
        db.drive(signed_speed, turn_rate)
    db.hold()

def PDtrack_and_junction(mm, speed, kp=0.1, kd=8, distance_forward=0, ):
    perror = 0
    db.reset()
    while (sensorL.reflection() > 5 or sensorR.reflection() > 5) and (db.distance() < (mm)):
        error = sensorL.reflection() - sensorR.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(-(speed + (p+d)))
        right_motor.dc(-(speed - (p+d)))
        perror = error
    db.straight(distance_forward)
    db.brake()

def PDtrack_or_junction(mm, speed, kp=0.5, kd=10, distance_forward=0):
    perror = 0
    db.reset()
    while (sensorL.reflection() > 20 and sensorR.reflection() > 20) and (db.distance() < (mm)):
        error = sensorL.reflection() - sensorR.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(-(speed + (p+d)))
        right_motor.dc(-(speed - (p+d)))
        perror = error
    db.straight(distance_forward)
    db.brake()

def PDtrack(mm, speed, kp=0.5, kd=10):
    perror = 0
    db.reset()
    signed_speed = speed if mm >= 0 else -speed
    while abs(db.distance()) < abs(mm):
        error = sensorL.reflection() - sensorR.reflection()
        p = error * kp
        d = (error - perror) * kd
        left_motor.dc(-(signed_speed + (p + d)))
        right_motor.dc(-(signed_speed - (p + d)))
        perror = error
    db.brake()

def line_square(speed=60, creep_speed=20, drop=25, timeout_ms=1500):
    """Squares the robot against a perpendicular black line by braking each
    wheel independently the instant ITS sensor sees reflection fall.
    Must be called while both sensors are still over a LIGHT background,
    approaching the line -- NOT after already sitting on top of it.

    Each wheel slows to creep_speed as it nears the line (before the actual
    stop threshold), so it's already moving slowly when it needs to brake --
    at full speed the wheel travels too far past the trigger before it can
    actually stop.
    """
    white_L = sensorL.reflection()
    white_R = sensorR.reflection()
    near_L = max(white_L - drop * 0.5, 5)
    near_R = max(white_R - drop * 0.5, 5)
    stop_L = max(white_L - drop, 5)
    stop_R = max(white_R - drop, 5)

    left_state = "approach"
    right_state = "approach"
    left_motor.dc(-speed)
    right_motor.dc(-speed)

    watch = StopWatch()
    while left_state != "done" or right_state != "done":
        if watch.time() > timeout_ms:
            break  # don't hang forever if a sensor never sees the drop
        rl = sensorL.reflection()
        rr = sensorR.reflection()

        if left_state == "approach" and rl <= near_L:
            left_motor.dc(-creep_speed)
            left_state = "creep"
        elif left_state == "creep" and rl <= stop_L:
            left_motor.brake()
            left_state = "done"

        if right_state == "approach" and rr <= near_R:
            right_motor.dc(-creep_speed)
            right_state = "creep"
        elif right_state == "creep" and rr <= stop_R:
            right_motor.brake()
            right_state = "done"

    left_motor.hold()
    right_motor.hold()

def align_zone():
    PDtrack_and_junction(900, 40, 1.3, 85)
    PDtrack(30, 40, 1.3, 85)
    PDtrack_and_junction(900, 40, 1.3, 85)
    straight_gyro(90, 100)
    turn_gyro(180)