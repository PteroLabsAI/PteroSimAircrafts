"""Fly the ZLT-NT airship by hand.

  python scripts/fly_airship.py --input keyboard
  python scripts/fly_airship.py --input gamepad

Spawn the airship in the editor first; this script only flies what is already there.

  command                              key           gamepad
  throttle, all three engines          W / S         left stick Y
  throttle - cut                       Space
  yaw: rudders and lateral thruster    A / D         left stick X
  elevator: nose up / down             Up / Down     right stick Y
  nacelles up / down                   Q / E         LT or RT
  ballonets - vent / fill              Z / X         buttons 1 / 2
  stop                                 Esc           Start

Throttle and nacelles are levers and hold where they are left; yaw and elevator spring
back. See README.md for what the ship does with all this.
"""

import argparse
import sys
import time

from pterosim import PteroSim

GRPC = "localhost:10010"
AIRCRAFT = "ZLT-NT"
RATE_HZ = 50.0

# Numbered as on the README diagram: 1 top, 2 right, 3 bottom, 4 left; Start is Options. On a
# DualSense under SDL2 those are 3 triangle, 1 circle, 0 cross, 2 square, and 6.
BTN_1, BTN_2, BTN_START = 3, 1, 6

# Controls.xml, in order.
DRIVE_STBD, DRIVE_PORT, DRIVE_AFT = 0, 1, 2
YAW, SIDE_SWIVEL, REAR_SWIVEL, ELEVATOR = 3, 4, 5, 6
BALLONET_FWD, BALLONET_AFT = 7, 8
CHANNELS = 9

# Straight up, and balanced: 0.75 of the side travel is 90 deg, 0.36 of the aft is 32. The
# aft one stops short on purpose -- equal thrust would be a two-to-one nose-down moment.
LIFT_SIDE, LIFT_AFT = 0.75, 0.36


def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


class Airship:
    """The nine channels, and the arithmetic that keeps them sane."""

    def __init__(self, aircraft):
        self.aircraft = aircraft
        self.c = [0.0] * CHANNELS

    def set(self, throttle=None, yaw=None, elevator=None, lift=None, ballonet=None):
        if throttle is not None:
            t = clamp(throttle, 0.0, 1.0)
            self.c[DRIVE_STBD] = self.c[DRIVE_PORT] = self.c[DRIVE_AFT] = t
        if yaw is not None:
            self.c[YAW] = clamp(yaw)
        if elevator is not None:
            self.c[ELEVATOR] = clamp(elevator)
        if lift is not None:
            l = clamp(lift, 0.0, 1.0)
            self.c[SIDE_SWIVEL] = LIFT_SIDE * l
            self.c[REAR_SWIVEL] = LIFT_AFT * l
        if ballonet is not None:
            self.c[BALLONET_FWD] = self.c[BALLONET_AFT] = clamp(ballonet)

    def push(self):
        self.aircraft.set_controls(self.c)

    def rest(self):
        self.c = [0.0] * CHANNELS
        self.push()


def connect():
    sim = PteroSim(GRPC)
    found = [a for a in sim.aircraft_status() if a.aircraft_name == AIRCRAFT]
    if not found:
        sys.exit("No %s in the scene -- spawn one in the editor first." % AIRCRAFT)

    aircraft = sim.get_aircraft(found[0].instance_id)
    # Nobody else at the controls: a connected autopilot writes the same channels at
    # 240 Hz and would win every other frame.
    try:
        aircraft.set_flight_stack("")
    except Exception as e:
        print("[note] could not take the autopilot off (%s); stop the simulation first if it fights back" % e)
    return sim, aircraft


KEYS = [
    ("W / S", "throttle up / down, all three engines"),
    ("Space", "throttle to zero"),
    ("A / D", "yaw left / right: rudders and the lateral thruster"),
    ("Up / Down", "elevator: nose up / down"),
    ("Q / E", "nacelles up / down -- Q to the balanced lift setting, E back to level"),
    ("Z / X", "ballonets: vent / fill (they trim the ship, they do not lift it)"),
    ("Esc", "stop"),
]

PAD = [
    ("left stick up/down", "throttle"),
    ("left stick left/right", "yaw"),
    ("right stick up/down", "elevator"),
    ("LT or RT", "nacelles: 0 is level flight, 1 is straight up and balanced"),
    ("buttons 1 / 2", "vent / fill the ballonets"),
    ("Start", "stop"),
]


def fly_manual(sim, ship, input_mode):
    """Drive the already-running simulation from one selected input device."""
    import pygame

    pygame.init()
    pygame.joystick.init()
    js = None
    if input_mode == "gamepad":
        if not pygame.joystick.get_count():
            sys.exit("No gamepad found. Connect one or use --input keyboard.")
        js = pygame.joystick.Joystick(0)
        js.init()
        print("stick: %s, %d axes, %d buttons" % (js.get_name(), js.get_numaxes(), js.get_numbuttons()))
    print()
    if input_mode == "keyboard":
        print("keyboard (click the window first):")
        for key, what in KEYS:
            print("    %-10s %s" % (key, what))
    if js:
        print()
        print("gamepad:")
        for key, what in PAD:
            print("    %-22s %s" % (key, what))
    print()

    screen = None
    if input_mode == "keyboard":
        screen = pygame.display.set_mode((620, 300))
        pygame.display.set_caption("ZLT-NT -- click here, then fly")
    font = pygame.font.SysFont("consolas", 15)
    big = pygame.font.SysFont("consolas", 17, bold=True)

    def axis(i, dead=0.06):
        if not js or i >= js.get_numaxes():
            return 0.0
        v = js.get_axis(i)
        return 0.0 if abs(v) < dead else v

    def held(keys, *names):
        return any(keys[getattr(pygame, "K_" + n)] for n in names)

    def triggers():
        return max((axis(4) + 1.0) * 0.5, (axis(5) + 1.0) * 0.5) if js and js.get_numaxes() > 5 else 0.0

    # Where the triggers rest, read once: a DualSense parks axes 4 and 5 at +1, and taken as a
    # lever position that put the nacelles straight up before anyone had touched anything.
    trig_rest = triggers()
    t0, next_print, throttle, lift = time.time(), 0.0, 0.0, 0.0
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT and screen:
                return
        keys = pygame.key.get_pressed()
        if (input_mode == "keyboard" and keys[pygame.K_ESCAPE]) or (js and js.get_numbuttons() > BTN_START and js.get_button(BTN_START)):
            return

        step = 1.0 / RATE_HZ
        if input_mode == "keyboard":
            throttle += (held(keys, "w") - held(keys, "s")) * 0.5 * step
            lift += (held(keys, "q") - held(keys, "e")) * 0.4 * step
            if keys[pygame.K_SPACE]:
                throttle = 0.0
        else:
            trig = triggers()
            if abs(trig - trig_rest) > 0.05:
                lift = trig
            if abs(axis(1)) > 0.0:
                throttle = (-axis(1) + 1.0) * 0.5
        throttle, lift = clamp(throttle, 0.0, 1.0), clamp(lift, 0.0, 1.0)

        if input_mode == "keyboard":
            yaw = clamp(held(keys, "d", "RIGHT") - held(keys, "a", "LEFT"))
            elevator = clamp(held(keys, "DOWN") - held(keys, "UP"))
            ballonet = clamp(held(keys, "x") - held(keys, "z"))
        else:
            yaw = axis(0)
            elevator = axis(3)
            ballonet = clamp(js.get_numbuttons() > max(BTN_1, BTN_2) and (js.get_button(BTN_2) - js.get_button(BTN_1)) or 0)

        ship.set(throttle=throttle, yaw=yaw, elevator=elevator, lift=lift, ballonet=ballonet)
        ship.push()

        a = next(a for a in sim.aircraft_status() if a.aircraft_name == AIRCRAFT)
        if screen:
            screen.fill((22, 26, 32))
            screen.blit(big.render("throttle %.2f   nacelles %.2f   yaw %+.1f   elevator %+.1f"
                                   % (throttle, lift, yaw, elevator), True, (235, 235, 230)), (14, 12))
            screen.blit(big.render("alt %6.1f m   pitch %+6.1f   heading %5.1f"
                                   % (a.z / 100.0, a.pitch, a.yaw), True, (150, 200, 255)), (14, 36))
            for i, (key, what) in enumerate(KEYS):
                screen.blit(font.render("%-10s %s" % (key, what), True, (170, 175, 180)), (14, 78 + i * 22))
            pygame.display.flip()

        if time.time() > next_print:
            print("  t+%5.1fs  alt=%7.2f m  pitch=%6.2f  yaw=%6.1f  thr=%.2f lift=%.2f"
                  % (time.time() - t0, a.z / 100.0, a.pitch, a.yaw, throttle, lift))
            next_print = time.time() + 1.0
        time.sleep(step)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", choices=("keyboard", "gamepad"), default="keyboard",
                    help="input device (default: keyboard)")
    args = ap.parse_args()

    sim, aircraft = connect()
    ship = Airship(aircraft)
    ship.rest()
    print("connected to the running simulation, %s at the controls\n" % args.input)

    try:
        fly_manual(sim, ship, args.input)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        ship.rest()


if __name__ == "__main__":
    main()
