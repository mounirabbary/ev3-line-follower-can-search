#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from time import sleep
import math

# ===================== DEVICES =====================
MLeft   = ev3.LargeMotor('outA')        # Left motor
MRight  = ev3.LargeMotor('outD')        # Right motor
clLeft  = ev3.ColorSensor('in3')        # Left color sensor on in3
clRight = ev3.ColorSensor('in4')        # Right color sensor on in4
US      = ev3.UltrasonicSensor('in2')   # Ultrasonic sensor on in2

assert MLeft.connected,   "Connect a motor to port A"
assert MRight.connected,  "Connect a motor to port D"
assert clRight.connected, "Connect a color sensor to port 4"
assert clLeft.connected,  "Connect a color sensor to port 3"
assert US.connected,      "Connect an ultrasonic sensor to port 2"

# ===================== LINE FOLLOWING PARAMETERS =====================

baseline    = -70  # duty cycle (0..100). If it goes backward/forward, flip sign or motor polarity
SIGN        = -1 if baseline < 0 else 1
SEARCHSPEED = -45  # pivot speed used when searching for the line

# Sensor modes
clRight.mode = 'COL-REFLECT'
clLeft.mode  = 'COL-REFLECT'
US.mode      = 'US-DIST-CM'
sleep(0.5)  # small initial pause

# ===================== PID =====================
P_GAIN = 3.2
I_GAIN = 0.1
D_GAIN = 0.1

last_error    = 0.0
integral      = 0.0
INTEGRAL_LIM  = 200.0
dt            = 0.01

soglia_pivot  = 25
soglia_bianco = 40  # > this value means "white"

# Start motors in direct mode
MLeft.run_direct()
MRight.run_direct()

# "slope parameters" reduced to min/max speed caps
min_mag  = 40
max_mag  = 80

# ===================== END-OF-LINE DETECTION =====================
line_lost_count  = 0
LINE_LOST_LIMIT  = 34   # ~0.3s of white-white before declaring "end of line"

can_found = False  # becomes True after the first can has been found

# ===================== CAN SEARCH PARAMETERS =====================

ARC_WIDE       = 14     # scan arc width in degrees
ARC_WIDE_STEP  = 2
ARC_FINE_HALF  = 10
ARC_FINE_STEP  = 2

CAN_MIN_CM      = 40.0
CAN_MAX_CM      = 99.0
US_MIN_VALID_CM = 3.0
US_MAX_VALID_CM = 250.0

K_TURN        = 6.0     # ticks per 1° of robot rotation
WHEEL_DIAM_MM = 56.0
DRIVE_SIGN    = -1      # keep consistent with your original code

SCAN_SPEED    = 180
PAUSE_READ    = 0.05

# ===================== ULTRASONIC HELPER =====================
def us_cm(samples=3, pause=0.03):
    vals = []
    for _ in range(samples):
        v = US.value()
        vals.append(v)
        sleep(pause)

    vals.sort()
    raw = vals[len(vals)//2]
    return (raw / 10.0) if raw > 200 else float(raw)

# ===================== MOTION HELPERS (CAN SEARCH) =====================
def drive_cm(cm, speed=200):
    circ_mm    = math.pi * WHEEL_DIAM_MM
    deg_per_mm = 360.0 / circ_mm
    mm         = cm * 10.0
    pos_sp     = int(DRIVE_SIGN * mm * deg_per_mm)
    MLeft.run_to_rel_pos(position_sp=pos_sp, speed_sp=speed)
    MRight.run_to_rel_pos(position_sp=pos_sp, speed_sp=speed)
    MLeft.wait_while('running')
    MRight.wait_while('running')

def turn_deg(deg, speed=SCAN_SPEED):
    ticks = int(deg * K_TURN)
    MLeft.run_to_rel_pos(position_sp= ticks, speed_sp=speed)
    MRight.run_to_rel_pos(position_sp=-ticks, speed_sp=speed)
    MLeft.wait_while('running')
    MRight.wait_while('running')

# ===================== ARC SCAN (INTERRUPTIBLE IF LINE IS FOUND) =====================
def scan_profile(a_min, a_max, step):
    profile = []
    current = 0
    for a in range(a_min, a_max + 1, step):

        # 🔴 BEHAVIOR CHECK: did we get back on the line during scanning?
        lv = clLeft.value()
        rv = clRight.value()
        if lv <= soglia_bianco or rv <= soglia_bianco:
            print("SCAN: line found during scan, exiting.")
            turn_deg(-current)  # return to center before exiting
            return profile, True  # True = scan aborted due to line

        # Normal scan step
        turn_deg(a - current)
        current = a

        d = us_cm()
        print("US:", d, flush=True)
        profile.append((a, d))
        sleep(PAUSE_READ)

    # End scan → return to center
    turn_deg(-current)
    return profile, False  # False = scan finished normally

# ===================== WINDOW DETECTION IN DISTANCE RANGE =====================
def find_window_in_range(profile, d_min, d_max):
    start = None
    end   = None
    was_in = False
    for ang, d in profile:
        valid    = (US_MIN_VALID_CM < d < US_MAX_VALID_CM)
        in_range = (d_min <= d <= d_max) if valid else False
        if in_range and not was_in and start is None:
            start = ang
        if was_in and not in_range and start is not None and end is None:
            end = ang
        was_in = in_range

    if start is not None and end is None:
        end = profile[-1][0]
    if start is None or end is None:
        return (None, None)
    return (start, end)

def aim_to_center_from_window(start_ang, end_ang):
    center = (start_ang + end_ang) / 2.0
    turn_deg(center)

def return_to_start(enc_start_L, enc_start_R, speed=250):
    dL = MLeft.position  - enc_start_L
    dR = MRight.position - enc_start_R

    if dL != 0 or dR != 0:
        SCALE = 0.6  # try 0.9 (90%), then adjust 0.85 / 0.95 if needed

        corrL = int(-dL * SCALE)
        corrR = int(-dR * SCALE)

        MLeft.run_to_rel_pos(position_sp=corrL, speed_sp=speed)
        MRight.run_to_rel_pos(position_sp=corrR, speed_sp=speed)
        MLeft.wait_while('running')
        MRight.wait_while('running')

    # Final re-orientation (as in your original code)
    turn_deg(110, speed=160)

# ===================== BEHAVIOR: CAN SEARCH (INTERRUPTIBLE) =====================
def search_can_behavior():
    global can_found
    ev3.Sound.beep()
    print("=== START CAN SEARCH ===")

    turn_deg(40, speed=160)

    # 1) Save starting encoder positions (end-of-line point)
    start_L = MLeft.position
    start_R = MRight.position

    # Temporarily stop direct motors
    MLeft.stop(stop_action='brake')
    MRight.stop(stop_action='brake')
    sleep(0.1)

    drive_cm(9, speed=160)
    sleep(0.2)

    # 2) WIDE SCAN: can be aborted if line is found again
    wide, aborted = scan_profile(ARC_WIDE, -ARC_WIDE, -ARC_WIDE_STEP)

    if aborted:
        print("CAN SEARCH: aborted, line found again.")
        # Restore direct mode to return to follower
        MLeft.run_direct()
        MRight.run_direct()
        return

    # 3) Is a can/object in range?
    a_start, a_end = find_window_in_range(wide, CAN_MIN_CM, CAN_MAX_CM)
    if a_start is None:
        ev3.Sound.tone([(400, 250, 50)])
        print("No object window found in wide scan.")
        MLeft.run_direct()
        MRight.run_direct()
        return

    print("Can window: start={}, end={}".format(a_start, a_end))

    aim_to_center_from_window(a_start, a_end)

    # Move a bit toward the can (short step)
    drive_cm(6, speed=160)
    sleep(0.2)

    # Return to the starting point (end-of-line point)
    return_to_start(start_L, start_R, speed=250)
    can_found = True

    # Re-enable direct mode for line follower
    MLeft.run_direct()
    MRight.run_direct()

    ev3.Sound.speak('Done')
    print("END CAN SEARCH, RETURNING TO THE LINE")

# ===================== MAIN LOOP: FOLLOW LINE + CAN BEHAVIOR =====================

print("Starting BEHAVIOR: follow line + search can (interruptible)")

while True:
    # ---- Read sensors ----
    left_val  = clLeft.value()
    right_val = clRight.value()

    # >>> DEBUG PRINT IF YOU WANT TO SEE SENSOR VALUES <<<
    # print("CL LEFT =", left_val, "  CL RIGHT =", right_val, flush=True)

    # Keep slope = True for PID capping logic (as in your code)
    slope = True

    # ---- Base speed (no gyro) ----
    base_mag = abs(baseline)
    if base_mag < min_mag:
        base_mag = min_mag
    if base_mag > max_mag:
        base_mag = max_mag
    BASE_SPEED = SIGN * base_mag

    # =============================================
    # BEHAVIOR: FOLLOW LINE + end-of-line detection
    # =============================================

    deap_eps = 10 if slope else 2  # deadband for error (typo kept from original: "deap")

    # ---- Gap/line logic with contrast between sensors ----
    diff = abs(left_val - right_val)
    CONTRAST_EPS = 10  # if needed: try 8–12

    # "True" white-white only if both are high AND similar
    is_real_gap = (left_val  > soglia_bianco and
                   right_val > soglia_bianco and
                   diff < CONTRAST_EPS)

    if is_real_gap:
        # === WHITE-WHITE GAP CASE ===
        line_lost_count += 1
        print("FOLLOW: WHITE-WHITE GAP, diff =", diff, " count =", line_lost_count)

        if line_lost_count < LINE_LOST_LIMIT or can_found:
            # still a "normal gap" → pivot to recover nearby line
            if last_error > 0:
                left_speed  =  SEARCHSPEED
                right_speed = -SEARCHSPEED
            else:
                left_speed  = -SEARCHSPEED
                right_speed =  SEARCHSPEED
        else:
            # consider the line finished → start can search
            print("END OF LINE DETECTED: STARTING CAN SEARCH")
            search_can_behavior()

            # reset to resume line control
            line_lost_count = 0
            integral        = 0
            last_error      = 0

            # after search, we are back at the end of the line
            left_speed  = BASE_SPEED
            right_speed = BASE_SPEED

    else:
        # Here we assume the line is still "near"
        # (at least one sensor sees dark OR there is strong contrast)
        line_lost_count = 0

        # ---- Normal PID line following ----
        error = left_val - right_val
        if abs(error) < deap_eps:
            error = 0

        integral += error * dt
        if integral > INTEGRAL_LIM:
            integral = INTEGRAL_LIM
        if integral < -INTEGRAL_LIM:
            integral = -INTEGRAL_LIM

        derivative     = (error - last_error) / dt
        correction_cmd = (P_GAIN * error) + (I_GAIN * integral) + (D_GAIN * derivative)

        if slope:
            cap = 0.4 * abs(BASE_SPEED)
            if correction_cmd >  cap:
                correction_cmd =  cap
            if correction_cmd < -cap:
                correction_cmd = -cap

        left_speed  = BASE_SPEED - correction_cmd
        right_speed = BASE_SPEED + correction_cmd

        left_speed  = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        # Keep direction consistent with BASE_SPEED sign
        if BASE_SPEED >= 0:
            left_speed  = max(0, left_speed)
            right_speed = max(0, right_speed)
        else:
            left_speed  = min(0, left_speed)
            right_speed = min(0, right_speed)

        last_error = error

    # ---- Apply speeds ----
    MLeft.duty_cycle_sp  = int(left_speed)
    MRight.duty_cycle_sp = int(right_speed)

    sleep(0.01)
