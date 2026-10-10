"""Fly Ingenuity by hand.

  python scripts/fly_ingenuity.py --input keyboard
  python scripts/fly_ingenuity.py --input gamepad

Spawn the ingenuity in the editor and start the simulation first; this script only flies what is already there.
Attitude is held by the model's own stability system (Systems/ingenuity_manual.xml): the sticks command a bank and a
pitch angle, a yaw rate and the collective lever; the script sends those five channels and the rotor switch.

  command                              key           gamepad
  collective lever up / down           W / S         left stick Y (position)
  yaw rate: nose left / right          A / D         left stick X
  pitch: nose down / up                Up / Down     right stick Y
  bank: left / right                   Left / Right  right stick X
  rotors on (spool up) / off           Enter / Bksp  buttons 1 / 2
  stop                                 Esc           Start

The lever holds where it is left; bank, pitch and yaw spring back to zero. Its bottom is the rotor's idle, just
under zero thrust. Take off with the lever a little above where it hovers (about 0.15 on Earth, 0.75 on Mars) and
bring it back once it lifts.
"""

import argparse
import os
import sys
import time

from pterosim import PteroSim

GRPC = "localhost:10010"
AIRCRAFT = "ingenuity"
RATE_HZ = 50.0

# Numbered as on the gamepad diagram in ZLT-NT/README.md: 1 top, 2 right; Start is Options. On a DualSense under
# SDL2 those are 3 triangle, 1 circle, and 6.
BTN_1, BTN_2, BTN_START = 3, 1, 6

# Controls.xml, in order: the six swashplate servos (the stability system writes them), one spare, the rotor speed
# control, then the manual channels.
RSC, ENABLE, ROLL, PITCH, YAW_RATE, COLLECTIVE = 7, 8, 9, 10, 11, 12
CHANNELS = 13

RSC_ON = 0.7               # H_RSC_SETPOINT 70 in firmwares/ardupilot_ingenuity.param: the governor's full speed
SPOOL_S = 3.0              # chosen: the RSC ramps over this, as ArduPilot's H_RSC_RUNUP_TIME would
LEVER_RATE = 0.25          # chosen: keyboard lever travel per second, full range in four seconds
# Lever 0 = root pitch just under zero thrust (-3.7 N on Earth, -0.1 N on Mars): 0 deg presses the legs in with 144 N,
# and at zero thrust itself FGRotor's inflow is singular on the ground (README.md, known limits).
IDLE_PITCH_DEG = 12.2
COLLECTIVE_RANGE_DEG = 25.0    # Systems/rotor_control.xml collective-range-rad
LEVER_IDLE = IDLE_PITCH_DEG / COLLECTIVE_RANGE_DEG


def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


class Heli:
    """The thirteen channels, and the ramp on the rotor switch."""

    def __init__(self, aircraft):
        self.aircraft = aircraft
        self.c = [0.0] * CHANNELS
        self.rotors = False

    def set(self, collective=None, roll=None, pitch=None, yaw=None, rotors=None):
        if collective is not None:
            self.c[COLLECTIVE] = LEVER_IDLE + clamp(collective, 0.0, 1.0) * (1.0 - LEVER_IDLE)
        if roll is not None:
            self.c[ROLL] = clamp(roll)
        if pitch is not None:
            self.c[PITCH] = clamp(pitch)
        if yaw is not None:
            self.c[YAW_RATE] = clamp(yaw)
        if rotors is not None:
            self.rotors = rotors

    def push(self):
        # The RSC ramps either way; the stability system is on whenever the script is at the controls.
        want = RSC_ON if self.rotors else 0.0
        self.c[RSC] += clamp(want - self.c[RSC], -RSC_ON / SPOOL_S / RATE_HZ, RSC_ON / SPOOL_S / RATE_HZ)
        self.c[ENABLE] = 1.0
        self.aircraft.set_controls(self.c)

    def rest(self):
        self.c = [0.0] * CHANNELS
        self.rotors = False
        self.aircraft.set_controls(self.c)


def connect():
    sim = PteroSim(GRPC)
    found = [a for a in sim.aircraft_status() if a.aircraft_name == AIRCRAFT]
    if not found:
        sys.exit("No %s in the scene -- spawn one in the editor first." % AIRCRAFT)
    aircraft = sim.get_aircraft(found[0].instance_id)
    # Nobody else at the controls: a connected autopilot writes the same channels at 240 Hz and would win.
    try:
        aircraft.set_flight_stack("")
    except Exception as e:
        print("[note] could not take the autopilot off (%s); stop the simulation first if it fights back" % e)
    return sim, aircraft


def status(sim, instance_id):
    return next(a for a in sim.aircraft_status() if a.instance_id == instance_id)


KEYS = [
    ("W / S", "collective lever up / down"),
    ("A / D", "yaw: nose left / right"),
    ("Up / Down", "pitch: nose down / up"),
    ("Left / Right", "bank left / right"),
    ("Enter / Bksp", "rotors on (spool up) / off"),
    ("Esc", "stop"),
]

PAD = [
    ("left stick up/down", "collective lever"),
    ("left stick left/right", "yaw rate"),
    ("right stick", "bank and pitch"),
    ("buttons 3 / 1", "rotors on / off (pygame numbering: Y / B on an Xbox pad)"),
    ("button 6", "stop (Back on an Xbox pad)"),
]


def fly_manual(sim, heli, input_mode):
    """Drive the already-running simulation from one selected input device."""
    # Without this SDL keeps only a pad's initial state while no window of ours has focus (gamepad mode opens none).
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
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
            print("    %-12s %s" % (key, what))
    if js:
        print()
        print("gamepad:")
        for key, what in PAD:
            print("    %-22s %s" % (key, what))
    print()

    screen = None
    if input_mode == "keyboard":
        screen = pygame.display.set_mode((640, 280))
        pygame.display.set_caption("ingenuity -- click here, then fly")
    font = pygame.font.SysFont("consolas", 15)
    big = pygame.font.SysFont("consolas", 17, bold=True)

    def axis(i, dead=0.06):
        if not js or i >= js.get_numaxes():
            return 0.0
        v = js.get_axis(i)
        return 0.0 if abs(v) < dead else v

    def held(keys, *names):
        return any(keys[getattr(pygame, "K_" + n)] for n in names)

    def button(i):
        return bool(js and js.get_numbuttons() > i and js.get_button(i))

    z0 = status(sim, heli.aircraft.instance_id).z / 100.0
    t0, next_print, lever = time.time(), 0.0, 0.0
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT and screen:
                return
        keys = pygame.key.get_pressed()
        if (input_mode == "keyboard" and keys[pygame.K_ESCAPE]) or button(BTN_START):
            return

        step = 1.0 / RATE_HZ
        if input_mode == "keyboard":
            lever += (held(keys, "w") - held(keys, "s")) * LEVER_RATE * step
            roll = clamp(held(keys, "RIGHT") - held(keys, "LEFT"))
            pitch = clamp(held(keys, "DOWN") - held(keys, "UP"))
            yaw = clamp(held(keys, "d") - held(keys, "a"))
            if keys[pygame.K_RETURN]:
                heli.set(rotors=True)
            if keys[pygame.K_BACKSPACE]:
                heli.set(rotors=False)
        else:
            if abs(axis(1)) > 0.0:
                lever = (-axis(1) + 1.0) * 0.5
            yaw = axis(0)
            roll = axis(2)
            pitch = -axis(3)
            if button(BTN_1):
                heli.set(rotors=True)
            if button(BTN_2):
                heli.set(rotors=False)
        lever = clamp(lever, 0.0, 1.0)

        heli.set(collective=lever, roll=roll, pitch=pitch, yaw=yaw)
        heli.push()

        a = status(sim, heli.aircraft.instance_id)
        if screen:
            screen.fill((22, 26, 32))
            screen.blit(big.render("lever %.2f   rotors %s (rsc %.2f)   bank %+.1f   pitch %+.1f   yaw %+.1f"
                                   % (lever, "ON" if heli.rotors else "off", heli.c[RSC], roll, pitch, yaw), True, (235, 235, 230)), (14, 12))
            screen.blit(big.render("agl %6.1f m   roll %+6.1f   pitch %+6.1f   heading %5.1f"
                                   % (a.z / 100.0 - z0, a.roll, a.pitch, a.yaw), True, (150, 200, 255)), (14, 36))
            for i, (key, what) in enumerate(KEYS):
                screen.blit(font.render("%-12s %s" % (key, what), True, (170, 175, 180)), (14, 78 + i * 22))
            pygame.display.flip()

        if time.time() > next_print:
            pressed = [i for i in range(js.get_numbuttons()) if js.get_button(i)] if js else []
            print("  t+%5.1fs  agl=%6.2f m  roll=%+5.1f pitch=%+5.1f heading=%5.1f  lever=%.2f rsc=%.2f  buttons=%s"
                  % (time.time() - t0, a.z / 100.0 - z0, a.roll, a.pitch, a.yaw, lever, heli.c[RSC], pressed))
            next_print = time.time() + 1.0
        time.sleep(step)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", choices=("keyboard", "gamepad"), default="keyboard",
                    help="input device (default: keyboard)")
    args = ap.parse_args()

    sim, aircraft = connect()
    heli = Heli(aircraft)
    heli.rest()
    print("connected to the running simulation, %s at the controls\n" % args.input)

    try:
        fly_manual(sim, heli, args.input)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        heli.rest()


if __name__ == "__main__":
    main()
