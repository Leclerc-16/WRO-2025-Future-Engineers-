"""
WRO Future Engineers – SPIKE Prime (hub API) – 3 laps + sign handling + parking

Port map
  A: Color Sensor (probe tip)
  B: Drive Motor (engine)
  C: Probe Arm Motor (extend/retract)  [down=215°, up=301°]
  D: Right-Side Distance Sensor
  E: Steering Motor (servo)            [0° = straight; + left, − right]
  F: Front Distance Sensor

This program is written for the LEGO SPIKE web IDE runtime (uses `import hub`).
To run: open https://spike.legoeducation.com/prime/project, connect hub, paste, run on hub.
To archive on GitHub: commit as e.g. `spike_hub_wro.py`.
"""

import hub
import time

# -----------------------------
# tiny helpers
# -----------------------------
def ms() -> int:
    return time.ticks_ms()

def wait_ms(t: int) -> None:
    time.sleep_ms(t)

# -----------------------------
# device handles
# -----------------------------
m_drive  = hub.port.B.motor
m_probe  = hub.port.C.motor
m_steer  = hub.port.E.motor

dev_front = hub.port.F.device
dev_side  = hub.port.D.device
dev_color = hub.port.A.device

# set sensor modes
try: dev_front.mode(0)   # distance (mm)
except: pass
try: dev_side.mode(0)    # distance (mm)
except: pass
try: dev_color.mode(5)   # RGB intensity (0..100) + (ambient) on some FW
except: pass

def get_front_mm() -> int:
    try:
        v = dev_front.get()
        return 99999 if (v is None or v[0] is None) else int(v[0])
    except:
        return 99999

def get_side_mm() -> int:
    try:
        v = dev_side.get()
        return 99999 if (v is None or v[0] is None) else int(v[0])
    except:
        return 99999

def get_rgb() -> tuple[int,int,int]:
    """Return (r,g,b) in 0..100 from color sensor A."""
    try:
        vals = dev_color.get()
        r, g, b = int(vals[0]), int(vals[1]), int(vals[2])
        return max(0,r), max(0,g), max(0,b)
    except:
        return (0,0,0)

# -----------------------------
# tunables (copy from Blocks values)
# -----------------------------
# probe angles
PROBE_DOWN_DEG = 215
PROBE_UP_DEG   = 301
PROBE_SPEED    = 600

# steering
STEER_SPEED_DPS  = 600
STEER_RIGHT_TURN = -45   # cornering
STEER_LEFT_TURN  = +45
STEER_RIGHT_SIGN = -30   # red pillar
STEER_LEFT_SIGN  = +30   # green pillar
STEER_WALL_NUDGE = +15

# timing
TURN_90_TIME_S   = 0.90
SIGN_HOLD_S      = 0.60
WALL_NUDGE_S     = 0.20

# drive
CRUISE_SPEED_DPS = 400
PARK_SPEED_DPS   = 300

# distance thresholds (mm)
CORNER_FRONT_MM  = 200   # front < 20 cm  => corner zone
CORNER_RIGHT_OPEN= 600   # right > 60 cm  => open to the right
CORNER_RIGHT_TIGHT = 200 # right < 20 cm  => tight corner (failsafe left)
PILLAR_FRONT_MM  = 250   # front < 25 cm  => likely pillar/sign
PILLAR_SIDE_CLEAR= 350   # right > 35 cm  => not hugging wall
WALL_TOO_NEAR_MM = 130   # right < 13 cm  => nudge away

# color voting
COLOR_SAMPLES    = 8
SAMPLE_DELAY_MS  = 20
MIN_VALUE_SUM    = 40    # ignore very dark reads

# -----------------------------
# motion helpers
# -----------------------------
def drive_forward(speed=CRUISE_SPEED_DPS):
    m_drive.run_at_speed(speed)

def drive_stop():
    m_drive.brake()

def steer_to(angle_deg: int):
    m_steer.run_to_position(angle_deg, STEER_SPEED_DPS)

def straighten():
    steer_to(0)

def steer_arc(angle_deg: int, hold_s: float):
    steer_to(angle_deg)
    time.sleep(hold_s)
    straighten()

def probe_to(angle_deg: int, speed_dps=PROBE_SPEED):
    m_probe.run_to_position(angle_deg, speed_dps)

def probe_down():
    probe_to(PROBE_DOWN_DEG)

def probe_up():
    probe_to(PROBE_UP_DEG)

# -----------------------------
# color classification via RGB
# -----------------------------
def hue_from_rgb(r,g,b):
    r_, g_, b_ = r/100.0, g/100.0, b/100.0
    mx, mn = max(r_,g_,b_), min(r_,g_,b_)
    d = mx - mn
    if d == 0:
        return None
    if mx == r_:
        h = ((g_ - b_) / d) % 6
    elif mx == g_:
        h = ((b_ - r_) / d) + 2
    else:
        h = ((r_ - g_) / d) + 4
    return 60*h  # 0..360

def in_window(h, center, win):
    if h is None: return False
    lo = (center - win) % 360
    hi = (center + win) % 360
    return (lo <= h <= hi) if lo <= hi else (h >= lo or h <= hi)

def classify_once():
    r,g,b = get_rgb()
    if (r+g+b) < MIN_VALUE_SUM:
        return None
    h = hue_from_rgb(r,g,b)
    # Hue windows: red≈0/360±20, green≈120±25, magenta≈300±25
    if in_window(h, 0, 20) or in_window(h, 360, 20):
        return 'red'
    if in_window(h, 120, 25):
        return 'green'
    if in_window(h, 300, 25):
        return 'magenta'
    # White marker (lap line): all high and balanced
    if r>60 and g>60 and b>60 and max(abs(r-g), abs(g-b), abs(r-b)) < 15:
        return 'white'
    return None

def vote_color(samples=COLOR_SAMPLES):
    counts = {'red':0,'green':0,'magenta':0,'white':0}
    for _ in range(samples):
        c = classify_once()
        if c in counts:
            counts[c] += 1
        wait_ms(SAMPLE_DELAY_MS)
    # pick top
    winner = max(counts, key=lambda k: counts[k])
    return winner if counts[winner] >= 2 else None

def see_white_marker() -> bool:
    # quick check for lap tile
    r,g,b = get_rgb()
    return (r>60 and g>60 and b>60 and max(abs(r-g), abs(g-b), abs(r-b)) < 15)

# -----------------------------
# behavior blocks
# -----------------------------
def handle_cornering(ud: int, sd: int) -> bool:
    """Returns True if a corner turn was performed."""
    # Right-turn primary: front blocked, right open
    if (ud < CORNER_FRONT_MM) and (sd > CORNER_RIGHT_OPEN):
        steer_arc(STEER_RIGHT_TURN, TURN_90_TIME_S)
        return True
    # Left-turn failsafe: both front and right tight
    if (ud < CORNER_FRONT_MM) and (sd < CORNER_RIGHT_TIGHT):
        steer_arc(STEER_LEFT_TURN, TURN_90_TIME_S)
        return True
    return False

def handle_wall_nudge(sd: int) -> bool:
    if sd < WALL_TOO_NEAR_MM:
        steer_arc(STEER_WALL_NUDGE, WALL_NUDGE_S)
        return True
    return False

def handle_pillar(ud: int, sd: int) -> bool:
    """Detect & react to red/green pillars. Returns True if handled."""
    if (ud < PILLAR_FRONT_MM) and (sd > PILLAR_SIDE_CLEAR):
        drive_stop()
        probe_down()
        wait_ms(120)
        sig = vote_color()
        probe_up()
        if   sig == 'green':
            steer_arc(STEER_LEFT_SIGN,  SIGN_HOLD_S)   # keep left of green
        elif sig == 'red':
            steer_arc(STEER_RIGHT_SIGN, SIGN_HOLD_S)   # keep right of red
        drive_forward()
        return True
    return False

def do_parking():
    """Wait for magenta, then enter and stop parallel."""
    drive_stop()
    # Wait until magenta seen
    while True:
        c = vote_color()
        if c == 'magenta':
            break
        wait_ms(30)
    # Enter the lot with a shallow left arc, then stop straight
    steer_to(+30)
    m_drive.run_at_speed(PARK_SPEED_DPS)
    time.sleep(1.5)  # tune for your car length
    drive_stop()
    straighten()

# -----------------------------
# main
# -----------------------------
def main():
    # startup posture
    probe_up()
    straighten()
    drive_stop()

    print("Press RIGHT hub button to start…")
    while not hub.button.right.is_pressed():
        wait_ms(30)
    while hub.button.right.is_pressed():
        wait_ms(30)

    # Drive
    drive_forward()

    laps_done = 0
    # We’ll require crossing the white marker with a short “cooldown”
    last_white_ms = -99999
    WHITE_COOLDOWN = 2000  # ms between valid lap detections

    while laps_done < 3:
        ud = get_front_mm()
        sd = get_side_mm()

        # 1) corners first
        if handle_cornering(ud, sd):
            pass
        # 2) wall keeping
        elif handle_wall_nudge(sd):
            pass
        # 3) pillar/sign
        else:
            handle_pillar(ud, sd)

        # lap detection on white tile (start/finish)
        if see_white_marker():
            now = ms()
            if now - last_white_ms > WHITE_COOLDOWN:
                laps_done += 1
                last_white_ms = now
                # small settle so we don't double count
                time.sleep(0.4)

        wait_ms(10)

    # After 3 laps → parking
    do_parking()
    print("Parked. Done.")

# entry
main()
