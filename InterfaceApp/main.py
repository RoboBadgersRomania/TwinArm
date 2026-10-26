"""TwinArm wireless interface: drive and watch the follower arm over WiFi through the LightGateway.

python main.py              connect from the Connection tab (gateway's own network: 192.168.4.1:4210)
python main.py 127.0.0.1    connect straight away, e.g. to python sim.py running in another terminal
"""
import json
import math
import os
import sys
import time

import pygame

from arm import fk, ik, wrap
from link import MODES, Link

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(HERE, "settings.json")

BG, PANEL, PANEL2, GRID = (11, 11, 11), (20, 16, 16), (34, 25, 25), (30, 23, 23)
LINE, DIM = (66, 44, 44), (120, 102, 94)
RED, RED_HI = (164, 22, 26), (200, 51, 58)
BEIGE, MUTED = (242, 232, 213), (150, 138, 124)
GREEN, AMBER = (96, 196, 120), (240, 170, 60)

JOINTS = [  # name, CAN driver id, graph colour; the first three are the side-view chain, the fourth turns on Q/E
    ("Shoulder", 1, (230, 72, 77)),
    ("Elbow", 2, (245, 165, 36)),
    ("Wrist pitch", 3, (46, 196, 182)),
    ("Wrist roll", 4, (120, 150, 245)),
]
TOP, SIDE, BAR = 60, 92, 64  # top bar height, sidebar width, control bar height

DEFAULTS = {
    "host": "192.168.4.1", "port": 4210,
    "L1": 300, "L2": 300, "L3": 142,            # mm: shoulder-elbow, elbow-wrist, wrist-tip
    "zero1": 80, "zero2": -132, "zero3": 52,    # joint angles as drawn at the arm zero (the idle pose photo)
    "sign1": -1, "sign2": 1, "sign3": -1, "sign4": -1, "sign_t": 1,  # motor direction against the drawing (checked on the arm)
    "table": 110,                               # mm from the shoulder axis down to the table: tip never goes below
    "limit": 150, "turret_limit": 90,           # +- degrees from the arm zero
    "turret_rate": 60, "roll_rate": 90,         # deg/s at 100 % while A/D or Q/E is held
    "rpm_max": 30,                              # joint move speed at 100 %, sent with Control ON (drivers forget it at power-off)
    "speed": 30,                                # %, the Speed field: scales joints, turret and roll
    "window": 20,                               # graph history, s
    "mirror": 0,                                # 1 = arm faces left, as in the photo
}
ROWS = [  # label, key[, step, min, max]; no step = toggle, key None = section title
    ("Arm geometry", None),
    ("Upper arm L1 (mm)", "L1", 1, 50, 800), ("Forearm L2 (mm)", "L2", 1, 50, 800),
    ("Wrist to tip L3 (mm)", "L3", 1, 10, 400), ("Table below shoulder (mm)", "table", 5, 0, 600),
    ("Shoulder at arm zero (°)", "zero1", 1, -180, 180), ("Elbow at arm zero (°)", "zero2", 1, -180, 180),
    ("Wrist at arm zero (°)", "zero3", 1, -180, 180),
    ("Motor direction", None),
    ("Shoulder", "sign1"), ("Elbow", "sign2"), ("Wrist pitch", "sign3"), ("Wrist roll", "sign4"), ("Turret", "sign_t"),
    ("Motion", None),
    ("Joint limit (± ° from zero)", "limit", 5, 10, 180), ("Turret limit (± °)", "turret_limit", 5, 10, 135),
    ("Turret speed at 100 %, A / D (°/s)", "turret_rate", 5, 5, 180),
    ("Wrist roll speed at 100 %, Q / E (°/s)", "roll_rate", 5, 5, 360),
    ("Joint speed at 100 % (rpm)", "rpm_max", 1, 1, 60),
    ("Display", None),
    ("Graph window (s)", "window", 5, 5, 120), ("Mirror view (arm faces left)", "mirror"),
]
CALIBRATION = {"zero1", "zero2", "zero3", "sign1", "sign2", "sign3", "sign4", "sign_t"}


def load():
    cfg = dict(DEFAULTS)
    try:
        with open(SETTINGS_FILE) as f:
            cfg.update(json.load(f))
    except (OSError, ValueError):
        pass
    return cfg


def save(cfg):
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except OSError:
        pass


_fonts = {}


def font(size, bold=False):
    if (size, bold) not in _fonts:
        _fonts[size, bold] = pygame.font.SysFont("segoeui,helveticaneue,helvetica,arial", size, bold)
    return _fonts[size, bold]


def text(surf, s, pos, size=14, color=BEIGE, bold=False, anchor="topleft"):
    img = font(size, bold).render(s, True, color)
    r = img.get_rect(**{anchor: pos})
    surf.blit(img, r)
    return r


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def icon_wifi(s, c, col):
    x, y = c
    for r in (13, 8.5):
        pygame.draw.arc(s, col, (x - r, y + 5 - r, 2 * r, 2 * r), math.radians(40), math.radians(140), 2)
    pygame.draw.circle(s, col, (x, y + 5), 2.5)


def icon_gear(s, c, col, hole):
    x, y = c
    pygame.draw.polygon(s, col, [(x + (10 if i % 4 < 2 else 7) * math.cos(i * math.pi / 16),
                                  y + (10 if i % 4 < 2 else 7) * math.sin(i * math.pi / 16)) for i in range(32)])
    pygame.draw.circle(s, hole, c, 3.5)


def icon_remote(s, c, col):
    x, y = c
    pygame.draw.rect(s, col, (x - 8, y - 6, 16, 24), 2, border_radius=5)
    pygame.draw.circle(s, col, (x, y + 2), 4, 2)
    for dx in (-3.5, 3.5):
        pygame.draw.circle(s, col, (x + dx, y + 12), 1.6)
    for r in (5, 9.5):
        pygame.draw.arc(s, col, (x - r, y - 9 - r, 2 * r, 2 * r), math.radians(45), math.radians(135), 2)


def icon_replay(s, c, col):
    x, y = c
    pygame.draw.arc(s, col, (x - 11, y - 11, 22, 22), math.radians(60), math.radians(360), 2)
    pygame.draw.polygon(s, col, [(x + 11, y - 6), (x + 6, y + 1), (x + 16, y + 1)])
    pygame.draw.polygon(s, col, [(x - 3, y - 5), (x - 3, y + 5), (x + 5, y)])


def icon_antenna(s, c, col):
    x, y = c
    pygame.draw.line(s, col, (x, y - 4), (x, y + 13), 2)
    pygame.draw.circle(s, col, (x, y - 5), 2.5)
    for r in (7, 12):
        for a in (135, -45):
            pygame.draw.arc(s, col, (x - r, y - 5 - r, 2 * r, 2 * r), math.radians(a), math.radians(a + 90), 2)


class App:
    def __init__(self, host=None):
        self.cfg = load()
        self.link = Link([j[1] for j in JOINTS])
        self.fields = {"host": host or self.cfg["host"], "port": str(self.cfg["port"]), "speed": str(self.cfg["speed"])}
        self.focus = self.panel = None
        self.tab = 0
        self.connected = self.control = self.drag = False
        self.g = self.zero()            # commanded joint angles as drawn
        self.tool = sum(self.g)         # tool angle kept while dragging, wheel changes it
        self.roll = self.turret = 0.0   # commanded, in the drawing's direction
        self.drag_mm = None
        self.ik_exact = True            # False while the drag target is out of reach or past a limit
        self.jams = set()               # CAN ids the Teensy reports jammed
        self.sent, self.full_at = {}, 0.0
        self.tick_at = self.hello_at = time.monotonic()
        self.toast_msg, self.toast_at, self.release_armed = "", 0.0, 0.0
        self.hits, self.view, self.tip_px, self.cursor, self.layer = [], (0, 0, 1, 1), None, None, None
        self.card_pick = {}             # chosen spot per info card, kept while it stays clear
        self.running = True
        if host:
            self.connect()

    # ---------- angles: drawn (side view) <-> motor (relative to the arm zero) ----------

    def zero(self):
        return [self.cfg["zero1"], self.cfg["zero2"], self.cfg["zero3"]]

    def lengths(self):
        return self.cfg["L1"], self.cfg["L2"], self.cfg["L3"]

    def drawn(self, i, motor_deg):
        return self.zero()[i] + self.cfg["sign%d" % (i + 1)] * motor_deg

    def motor(self, i, drawn_deg):
        return self.cfg["sign%d" % (i + 1)] * wrap(drawn_deg - self.zero()[i])

    def measured(self):
        if not self.link.robot_online():
            return None
        return [self.drawn(i, self.link.motors[JOINTS[i][1]].deg) for i in range(3)]

    def to_screen(self, p):
        ox, oy, s, d = self.view
        return ox + d * p[0] * s, oy - p[1] * s

    def to_mm(self, q):
        ox, oy, s, d = self.view
        return (q[0] - ox) / (s * d), (oy - q[1]) / s

    # ---------- control ----------

    def toast(self, msg):
        self.toast_msg, self.toast_at = msg, time.monotonic()

    def set_control(self, on, why=""):
        if on and not self.link.robot_online():
            self.toast("No telemetry from the arm: connect first")
            return
        if on and self.jams:
            self.toast("A joint is jammed: free it, then Clear jam")
            return
        self.control, self.drag = on, False
        if on:  # start from where the arm is, so nothing jumps
            self.g = self.measured()
            self.tool = sum(self.g)
            self.roll = self.cfg["sign4"] * self.link.motors[JOINTS[3][1]].deg
            self.sent.clear()
            self.full_at = 0.0
            self.link.send(*self.speed_lines())
        self.toast(why or ("Control ON: drag the red tip" if on else "Control OFF: the arm holds its pose"))

    def solve(self, tip):
        """Commanded joints for a tip target: inside the joint limits, the whole arm above the table."""
        floor, lim = 15 - self.cfg["table"], self.cfg["limit"]
        g, self.ik_exact = ik((tip[0], max(tip[1], floor)), self.tool, self.lengths(),
                              [(z - lim, z + lim) for z in self.zero()], self.g)
        if min(p[1] for p in fk(g, self.lengths())[1:]) >= floor:  # elbow or wrist would hit the table: stay put
            self.g = g

    def control_tick(self, dt):
        if not self.control:
            return
        if not self.link.robot_online():
            self.set_control(False, "Telemetry lost: control OFF, the arm holds its last target")
            return
        if self.focus is None:
            k = pygame.key.get_pressed()
            tl = self.cfg["turret_limit"]
            pct = self.cfg["speed"] / 100
            self.turret = clamp(self.turret + (k[pygame.K_a] - k[pygame.K_d]) * self.cfg["turret_rate"] * pct * dt, -tl, tl)
            self.roll = clamp(self.roll + (k[pygame.K_e] - k[pygame.K_q]) * self.cfg["roll_rate"] * pct * dt,
                              -self.cfg["limit"], self.cfg["limit"])
        targets = {JOINTS[i][1]: self.motor(i, self.g[i]) for i in range(3)}
        targets[JOINTS[3][1]] = self.cfg["sign4"] * self.roll
        targets["t"] = self.cfg["sign_t"] * self.turret
        full = time.monotonic() - self.full_at > 0.5  # resend everything twice a second in case a datagram got lost
        lines = []
        for key, v in targets.items():
            if full or abs(v - self.sent.get(key, 1e9)) > 0.05:
                lines.append("t %.1f" % v if key == "t" else "%d a%.2f" % (key, v))
                self.sent[key] = v
        if full:
            self.full_at = time.monotonic()
        self.link.send(*lines)

    def connect(self):
        host, port = self.fields["host"].strip(), self.fields["port"].strip()
        if not port.isdigit() or not 0 < int(port) < 65536:
            self.toast("Port must be a number from 1 to 65535")
            return
        self.cfg.update(host=host, port=int(port))
        save(self.cfg)
        self.link.connect(host, int(port))
        self.connected, self.hello_at = True, time.monotonic()
        self.toast("Talking to the gateway at %s:%s" % (host, port))

    def toggle_connect(self):
        if not self.connected:
            self.connect()
            return
        if self.control:
            self.set_control(False)
        self.link.close()
        self.connected = False
        self.toast("Disconnected")

    def speed_lines(self):
        """Joint speed for every driver: Speed % of the 100 % rpm."""
        rpm = max(0.1, self.cfg["rpm_max"] * self.cfg["speed"] / 100)
        return ["%d v%.2f" % (j[1], rpm) for j in JOINTS]

    def apply_speed(self):
        s = self.fields["speed"].strip()
        value = int(clamp(int(s), 1, 100)) if s.isdigit() else self.cfg["speed"]
        self.fields["speed"] = str(value)
        if value != self.cfg["speed"]:
            self.change("speed", value)
            self.toast("Speed %d %%" % value)

    def home(self):
        self.set_control(False, "Homing all joints: telemetry pauses until it is done")
        self.link.send("h", *self.speed_lines())  # homing sets its own speed; these run once it is done

    def idle(self):
        """Back to the pose homing found: 0 is the arm zero the Teensy stored, so no re-homing."""
        self.turret = self.roll = 0.0
        if self.control:  # move the commanded pose, the control loop sends it
            self.g, self.drag = self.zero(), False
            self.tool = sum(self.g)
        else:
            self.link.send(*self.speed_lines() + ["%d a0" % j[1] for j in JOINTS] + ["t 0"])
        self.toast("Going to the idle pose found by homing")

    def clear_faults(self):
        self.link.send(*("%d c" % j[1] for j in JOINTS))  # clears driver faults and jams
        self.toast("Fault and jam clear sent to every driver")

    def release(self):
        if time.monotonic() - self.release_armed > 2:
            self.release_armed = time.monotonic()
            self.toast("Click again within 2 s: every motor goes limp and the arm falls under gravity")
            return
        self.release_armed = 0.0
        self.set_control(False, "All motors released")
        self.link.send("x")

    def change(self, key, value):
        if self.control and key in CALIBRATION:
            self.set_control(False, "Control OFF: calibration changed")
        self.cfg[key] = value
        save(self.cfg)
        if key in ("speed", "rpm_max") and self.connected:
            self.link.send(*self.speed_lines())

    def bump(self, key, step, lo, hi):
        mult = 10 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 1
        self.change(key, clamp(self.cfg[key] + step * mult, lo, hi))

    def flip(self, key):
        self.change(key, -self.cfg[key] if key.startswith("sign") else 1 - self.cfg[key])

    def reset(self):
        if self.control:
            self.set_control(False, "Control OFF: settings reset")
        self.cfg = dict(DEFAULTS, host=self.cfg["host"], port=self.cfg["port"])
        self.fields["speed"] = str(self.cfg["speed"])
        save(self.cfg)

    def toggle_panel(self, name):
        self.panel = None if self.panel == name else name

    # ---------- events ----------

    def event(self, e):
        if e.type == pygame.QUIT:
            self.running = False
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            if self.focus == "speed":  # clicking away applies what was typed
                self.apply_speed()
            self.focus = None
            for rect, fn in reversed(self.hits):  # last drawn is on top
                if rect.collidepoint(e.pos):
                    if fn:
                        fn()
                    return
            if self.panel:
                self.panel = None
            elif self.control and self.tip_px and math.dist(e.pos, self.tip_px) < 24:
                self.drag, self.drag_mm = True, self.to_mm(e.pos)
        elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
            self.drag = False
        elif e.type == pygame.MOUSEMOTION and self.drag:
            self.drag_mm = self.to_mm(e.pos)
            self.solve(self.drag_mm)
        elif e.type == pygame.MOUSEWHEEL and self.control:
            self.tool = wrap(self.tool + 3 * e.y)
            self.solve(self.drag_mm if self.drag else fk(self.g, self.lengths())[-1])
        elif e.type == pygame.TEXTINPUT and self.focus:
            if self.focus == "host" or e.text.isdigit():
                self.fields[self.focus] = (self.fields[self.focus] + e.text)[:3 if self.focus == "speed" else 64]
        elif e.type == pygame.KEYDOWN:
            self.key(e.key)

    def key(self, k):
        if self.focus:
            if k == pygame.K_BACKSPACE:
                self.fields[self.focus] = self.fields[self.focus][:-1]
            elif k in (pygame.K_RETURN, pygame.K_KP_ENTER):
                if self.focus == "speed":
                    self.apply_speed()
                else:
                    self.connect()
                self.focus = None
            elif k == pygame.K_TAB and self.focus != "speed":
                self.focus = "port" if self.focus == "host" else "host"
            elif k == pygame.K_ESCAPE:
                if self.focus == "speed":
                    self.fields["speed"] = str(self.cfg["speed"])  # Esc drops the edit
                self.focus = None
        elif k == pygame.K_SPACE:
            self.set_control(not self.control)
        elif k == pygame.K_i:
            self.idle()
        elif k == pygame.K_ESCAPE:
            if self.panel:
                self.panel = None
            elif self.control:
                self.set_control(False)

    def update(self):
        self.link.poll()
        jams = {j[1] for j in JOINTS if self.link.motors[j[1]].jam}
        new = [j for j in JOINTS if j[1] in jams - self.jams]
        self.jams = jams
        if new:  # the Teensy already holds the joint; stop streaming targets into the obstacle
            msg = "%s jammed at %+.1f°: holding there. Free it, then Clear jam" % (
                new[0][0], self.link.motors[new[0][1]].deg)
            if self.control:
                self.set_control(False, msg)
            else:
                self.toast(msg)
        now = time.monotonic()
        if self.connected and now - self.hello_at > 1.0:
            self.link.hello()
            self.hello_at = now
        if now - self.tick_at >= 0.05:  # commands at 20 Hz
            self.control_tick(min(0.2, now - self.tick_at))
            self.tick_at = now

    # ---------- drawing ----------

    def status(self):
        if not self.connected:
            return MUTED, "Not connected"
        if self.link.robot_online():
            return GREEN, "Arm online · %.0f Hz telemetry" % self.link.rate()
        if self.link.gateway_online():
            return AMBER, "Gateway online · no arm telemetry"
        return RED_HI, "Waiting for the gateway…"

    def button(self, surf, rect, label, fn, kind="plain", size=14, bold=False):
        fill, edge, ink = {"plain": (PANEL2, LINE, BEIGE), "on": (RED, RED_HI, BEIGE),
                           "danger": (PANEL2, RED, RED_HI)}[kind]
        if rect.collidepoint(pygame.mouse.get_pos()):
            fill = tuple(min(255, c + 16) for c in fill)
        pygame.draw.rect(surf, fill, rect, border_radius=8)
        pygame.draw.rect(surf, edge, rect, 1, border_radius=8)
        text(surf, label, rect.center, size, ink, bold, "center")
        self.hits.append((rect, fn))

    def tab_button(self, surf, rect, active, fn):
        fill = PANEL2 if active or rect.collidepoint(pygame.mouse.get_pos()) else PANEL
        pygame.draw.rect(surf, fill, rect, border_radius=10)
        pygame.draw.rect(surf, RED if active else LINE, rect, 1, border_radius=10)
        self.hits.append((rect, fn))
        return fill

    def panel_bg(self, surf, r):
        pygame.draw.rect(surf, (0, 0, 0), r.move(0, 6), border_radius=14)
        pygame.draw.rect(surf, PANEL2, r, border_radius=14)
        pygame.draw.rect(surf, RED, r, 1, border_radius=14)
        self.hits.append((r, None))  # swallow clicks on the panel background

    def draw(self, surf):
        self.hits = []
        W, H = surf.get_size()
        surf.fill(BG)
        content = pygame.Rect(0, TOP, W - SIDE, H - TOP)
        self.draw_manual(surf, content)
        self.draw_sidebar(surf, pygame.Rect(W - SIDE, TOP, SIDE, H - TOP))
        self.draw_top(surf, W)
        if self.panel == "conn":
            self.draw_conn(surf)
        elif self.panel == "settings":
            self.draw_settings(surf, W)
        self.draw_toast(surf, content)
        self.update_cursor()

    def draw_top(self, surf, W):
        pygame.draw.rect(surf, PANEL, (0, 0, W, TOP))
        pygame.draw.line(surf, RED, (0, TOP - 1), (W, TOP - 1), 2)
        col, status = self.status()
        r = pygame.Rect(12, 10, 300, 40)
        self.tab_button(surf, r, self.panel == "conn", lambda: self.toggle_panel("conn"))
        icon_wifi(surf, (r.x + 22, r.centery - 3), col)
        text(surf, "Connection", (r.x + 44, r.y + 2), 15, BEIGE, True)
        text(surf, status, (r.x + 44, r.y + 21), 12, col)

        parts = [font(22, True).render("TWIN", True, BEIGE), font(22, True).render("ARM", True, RED_HI),
                 font(13).render("   WIRELESS INTERFACE", True, MUTED)]
        x = W // 2 - sum(p.get_width() for p in parts) // 2
        for p in parts:
            surf.blit(p, p.get_rect(midleft=(x, TOP // 2)))
            x += p.get_width()

        r = pygame.Rect(W - 162, 10, 150, 40)
        fill = self.tab_button(surf, r, self.panel == "settings", lambda: self.toggle_panel("settings"))
        icon_gear(surf, (r.x + 24, r.centery), BEIGE, fill)
        text(surf, "Settings", (r.x + 44, r.centery), 15, BEIGE, True, "midleft")

    def draw_sidebar(self, surf, r):
        pygame.draw.rect(surf, PANEL, r)
        pygame.draw.line(surf, LINE, r.topleft, r.bottomleft)
        tabs = [("Manual", "Telemetry", icon_remote, None),
                ("Teach &", "Replay", icon_replay, "Teach & Replay comes with the recording firmware"),
                ("LoRa", "Master", icon_antenna, "The LoRa master arm comes after the radio link")]
        for i, (l1, l2, icon, later) in enumerate(tabs):
            t = pygame.Rect(r.x + 8, r.y + 12 + i * 96, r.w - 16, 86)
            active = i == self.tab
            if active:
                pygame.draw.rect(surf, PANEL2, t, border_radius=10)
                pygame.draw.rect(surf, RED_HI, (r.x, t.y + 12, 3, t.h - 24))
            col = BEIGE if active else (DIM if later else MUTED)
            icon(surf, (t.centerx, t.y + 27), col)
            text(surf, l1, (t.centerx, t.y + 54), 11, col, active, "center")
            text(surf, l2, (t.centerx, t.y + 68), 11, col, active, "center")
            self.hits.append((t, (lambda m=later: self.toast(m)) if later else (lambda i=i: setattr(self, "tab", i))))

    def draw_manual(self, surf, c):
        text(surf, "Manual Wireless Telemetry", (c.x + 18, c.y + 12), 20, BEIGE, True)
        text(surf, "Control ON, then drag the red tip  ·  wheel: tool angle  ·  A / D: turret  ·  Q / E: wrist roll"
                   "  ·  I: idle  ·  Space: control  ·  Esc: off", (c.x + 18, c.y + 40), 12, MUTED)
        body = pygame.Rect(c.x + 16, c.y + 64, c.w - 32, c.h - 80)
        gw = max(320, int(body.w * 0.34))
        self.draw_arm(surf, pygame.Rect(body.x, body.y, body.w - gw - 14, body.h))
        self.draw_graphs(surf, pygame.Rect(body.right - gw, body.y, gw, body.h))

    def draw_arm(self, surf, c):
        pygame.draw.rect(surf, PANEL, c, border_radius=14)
        area = pygame.Rect(c.x, c.y, c.w, c.h - BAR)
        L = self.lengths()
        R = sum(L)
        s = max(0.05, min((area.w - 60) / (1.4 * R), (area.h - 50) / (1.15 * R + self.cfg["table"])))
        d = -1 if self.cfg["mirror"] else 1
        ox, oy = area.centerx - d * 0.3 * R * s, area.y + 25 + 1.0 * R * s
        self.view = (ox, oy, s, d)

        surf.set_clip(area.inflate(-4, -4))
        for k in range(-20, 21):  # 100 mm grid
            x, y = ox + k * 100 * s, oy - k * 100 * s
            pygame.draw.line(surf, GRID, (x, area.top), (x, area.bottom))
            pygame.draw.line(surf, GRID, (area.left, y), (area.right, y))
        pygame.draw.circle(surf, LINE, (ox, oy), R * s, 1)  # full reach
        fy = oy + self.cfg["table"] * s
        pygame.draw.rect(surf, (24, 18, 18), (area.left, fy, area.w, area.bottom - fy))
        pygame.draw.line(surf, LINE, (area.left, fy), (area.right, fy), 2)
        pygame.draw.polygon(surf, (44, 32, 32), [(ox - 75 * s, fy), (ox + 75 * s, fy), (ox + 40 * s, oy), (ox - 40 * s, oy)])

        meas = self.measured()
        if self.layer is None or self.layer.get_size() != area.size:
            self.layer = pygame.Surface(area.size, pygame.SRCALPHA)
        self.layer.fill((0, 0, 0, 0))
        if self.control:
            if meas:
                self.draw_pose(self.layer, area.topleft, meas, RED_HI, 95)  # where the arm really is
            pose = self.g
            self.draw_pose(self.layer, area.topleft, pose, BEIGE, 255)
        else:  # idle: follows the telemetry and breathes
            pose = meas or self.zero()
            self.draw_pose(self.layer, area.topleft, pose, BEIGE, int(120 + 60 * math.sin(time.monotonic() * 2.2)))
        surf.blit(self.layer, area.topleft)

        pts = [self.to_screen(p) for p in fk(pose, L)]
        for i, p in enumerate(pts[:3]):
            r = (24, 20, 16)[i] * s + 4
            pygame.draw.circle(surf, BG, p, r)
            pygame.draw.circle(surf, JOINTS[i][2], p, r, 3)
        for i, (_, cid, _) in enumerate(JOINTS):  # wrist roll pulses at the tip
            if cid in self.jams:
                self.draw_jam(surf, pts[i])
        tip = self.tip_px = pts[3]
        if self.control:
            if self.drag and self.drag_mm:
                q = self.to_screen(self.drag_mm)
                if math.dist(q, tip) > 4:
                    col = RED_HI if self.ik_exact else AMBER
                    pygame.draw.line(surf, col, tip, q, 1)
                    pygame.draw.circle(surf, col, q, 6, 1)
            big = self.drag or math.dist(pygame.mouse.get_pos(), tip) < 24
            pygame.draw.circle(surf, RED_HI, tip, 13 if big else 10)
            pygame.draw.circle(surf, BEIGE, tip, 13 if big else 10, 2)
        else:
            pygame.draw.circle(surf, DIM, tip, 7, 2)

        tx = area.left + 62 if d == 1 else area.right - 62
        self.draw_turret(surf, (tx, area.y + 64))
        dial = pygame.Rect(tx - 64, area.y + 22, 128, 110)
        for i, r in enumerate(self.place_cards(area, pts, [dial])):  # the roll card sits by the tip
            self.card(surf, i, pts[i], r)
        if self.control and self.drag and not self.ik_exact:
            text(surf, "Target out of reach or past a joint limit: holding the nearest pose",
                 (area.centerx, area.y + 18), 13, AMBER, anchor="center")
        if not meas:
            text(surf, "No arm telemetry: showing the arm zero" if self.connected else "Not connected: showing the arm zero",
                 (area.centerx, area.y + 18), 13, MUTED, anchor="center")
        surf.set_clip(None)

        bar = pygame.Rect(c.x, c.bottom - BAR, c.w, BAR)
        pygame.draw.line(surf, LINE, (bar.x + 14, bar.y), (bar.right - 14, bar.y))
        x, y, h = bar.x + 14, bar.y + 12, 40
        self.button(surf, pygame.Rect(x, y, 176, h), "CONTROL  ON" if self.control else "CONTROL  OFF",
                    lambda: self.set_control(not self.control), "on" if self.control else "plain", 15, True)
        x += 186
        self.button(surf, pygame.Rect(x, y, 80, h), "Idle", self.idle)
        x += 90
        self.button(surf, pygame.Rect(x, y, 96, h), "Home all", self.home)
        x += 106
        self.button(surf, pygame.Rect(x, y, 112, h), "Clear jam" if self.jams else "Clear faults", self.clear_faults,
                    "on" if self.jams else "plain")
        x += 122
        armed = time.monotonic() - self.release_armed < 2
        self.button(surf, pygame.Rect(x, y, 146, h), "Click to confirm" if armed else "Release motors", self.release, "danger")
        sf = pygame.Rect(bar.right - 14 - 76, y, 76, h)  # Speed %, 1-100: Enter or click away applies it
        self.field(surf, sf, "speed")
        text(surf, "%", (sf.right - 12, sf.centery), 15, MUTED, anchor="midright")
        text(surf, "Speed", (sf.x - 10, sf.centery), 13, BEIGE, True, "midright")

    def draw_pose(self, layer, off, g, col, alpha):
        s = self.view[2]
        pts = [(x - off[0], y - off[1]) for x, y in map(self.to_screen, fk(g, self.lengths()))]
        rgba = (*col, alpha)
        (wx, wy), (tx, ty) = pts[2], pts[3]
        n = math.hypot(tx - wx, ty - wy) or 1
        ux, uy = (tx - wx) / n, (ty - wy) / n
        palm = (tx - 26 * s * ux, ty - 26 * s * uy)
        for (a, b), w in zip([(pts[0], pts[1]), (pts[1], pts[2]), (pts[2], palm)], (64, 50, 38)):
            w = max(4, int(w * s))
            pygame.draw.line(layer, rgba, a, b, w)
            pygame.draw.circle(layer, rgba, a, w / 2)
            pygame.draw.circle(layer, rgba, b, w / 2)
        for side in (-1, 1):  # gripper jaws
            jx, jy = -uy * side * 13 * s, ux * side * 13 * s
            pygame.draw.line(layer, rgba, (palm[0] + jx, palm[1] + jy), (tx + jx * 0.6, ty + jy * 0.6), max(3, int(9 * s)))

    def place_cards(self, area, pts, avoid):
        """One card spot next to each of pts, off the arm, the other cards and `avoid`."""
        arm = [(a[0] + (b[0] - a[0]) * t / 10, a[1] + (b[1] - a[1]) * t / 10) for a, b in zip(pts, pts[1:]) for t in range(11)]
        placed, out = list(avoid), []
        for i, p in enumerate(pts):
            best = None
            for k in range(16):
                for dist in (1.0, 1.4, 1.9):
                    r = pygame.Rect(0, 0, 176, 78)
                    r.center = (p[0] + math.cos(k * math.pi / 8) * 120 * dist, p[1] + math.sin(k * math.pi / 8) * 70 * dist)
                    r.clamp_ip(area.inflate(-12, -12))
                    hit = r.inflate(36, 36)
                    cost = (sum(hit.collidepoint(q) for q in arm) + 60 * sum(hit.collidepoint(q) for q in pts)  # never hide a joint or the tip
                            + 40 * sum(hit.colliderect(o) for o in placed) + dist
                            + (0 if self.card_pick.get(i) == (k, dist) else 1.5))  # stay put unless it gets crowded
                    if best is None or cost < best[0]:
                        best = (cost, r, (k, dist))
            self.card_pick[i] = best[2]
            placed.append(best[1])
            out.append(best[1])
        return out

    def draw_jam(self, surf, p):
        """Two red ripples running out of a jammed joint and a throbbing core."""
        t = time.monotonic()
        for k in (0, 0.5):
            ph = (t * 1.2 + k) % 1
            r = 16 + 38 * ph
            ring = pygame.Surface((2 * r + 6, 2 * r + 6), pygame.SRCALPHA)
            pygame.draw.circle(ring, (*RED_HI, int(230 * (1 - ph))), (r + 3, r + 3), r, 4)
            surf.blit(ring, (p[0] - r - 3, p[1] - r - 3))
        pygame.draw.circle(surf, (int(150 + 90 * math.sin(t * 9)), 18, 24), p, 11)
        pygame.draw.circle(surf, BEIGE, p, 11, 2)

    def card(self, surf, i, p, r):
        name, cid, col = JOINTS[i]
        m = self.link.motors[cid]
        online = m.ok and time.monotonic() - m.seen < 1.0
        pygame.draw.line(surf, col, p, (clamp(p[0], r.left, r.right), clamp(p[1], r.top, r.bottom)), 1)
        pygame.draw.rect(surf, PANEL2, r, border_radius=10)
        if m.jam:
            pulse = int(140 + 115 * abs(math.sin(time.monotonic() * 4)))
            pygame.draw.rect(surf, (pulse, 30, 36), r, 2, border_radius=10)
            text(surf, "JAMMED", (r.right - 10, r.y + 5), 11, RED_HI, True, "topright")
        else:
            pygame.draw.rect(surf, col, r, 1, border_radius=10)
            text(surf, "#%d" % cid, (r.right - 10, r.y + 5), 11, MUTED, anchor="topright")
        text(surf, name, (r.x + 10, r.y + 5), 12, col, True)
        if not online:
            text(surf, "offline", (r.x + 10, r.y + 26), 18, DIM, True)
            return
        text(surf, "%+.1f°" % m.deg, (r.x + 10, r.y + 20), 20, BEIGE, True)
        if m.mode:
            text(surf, "→ %+.1f°" % m.target, (r.right - 10, r.y + 27), 12, MUTED, anchor="topright")
        tcol = RED_HI if m.temp >= 75 else AMBER if m.temp >= 60 else MUTED
        text(surf, "%d °C" % m.temp, (r.x + 10, r.y + 46), 12, tcol)
        text(surf, "%.1f W   %.2f A" % (m.watts, m.amps), (r.right - 10, r.y + 46), 12, MUTED, anchor="topright")
        text(surf, MODES[m.mode] if m.mode < len(MODES) else "?", (r.x + 10, r.y + 61), 11, MUTED)
        if m.fault:
            text(surf, "fault: " + m.faults, (r.right - 10, r.y + 61), 11, RED_HI, True, "topright")

    def draw_turret(self, surf, c):
        x, y = c
        r = 34
        pygame.draw.circle(surf, PANEL2, c, r)
        pygame.draw.circle(surf, LINE, c, r, 1)
        lim = math.radians(self.cfg["turret_limit"])
        pygame.draw.arc(surf, DIM, (x - r + 4, y - r + 4, 2 * r - 8, 2 * r - 8), math.pi / 2 - lim, math.pi / 2 + lim, 2)
        a = math.radians(self.turret)
        pygame.draw.line(surf, RED_HI, c, (x - math.sin(a) * (r - 7), y - math.cos(a) * (r - 7)), 4)
        pygame.draw.circle(surf, BEIGE, c, 5)
        k = pygame.key.get_pressed()
        typing = self.focus is not None
        text(surf, "A", (x - r - 14, y), 14, BEIGE if self.control and k[pygame.K_a] and not typing else DIM, True, "center")
        text(surf, "D", (x + r + 14, y), 14, BEIGE if self.control and k[pygame.K_d] and not typing else DIM, True, "center")
        text(surf, "Turret %+.1f°" % self.turret, (x, y + r + 12), 13, BEIGE, True, "center")
        text(surf, "top view · open loop", (x, y + r + 28), 11, DIM, anchor="center")

    def draw_graphs(self, surf, r):
        legend = pygame.Rect(r.x, r.y, r.w, 38 + 22 * len(JOINTS))
        pygame.draw.rect(surf, PANEL, legend, border_radius=12)
        text(surf, "Motors", (legend.x + 14, legend.y + 10), 14, BEIGE, True)
        text(surf, "tracking error  ·  current", (legend.right - 14, legend.y + 12), 11, MUTED, anchor="topright")
        for i, (name, cid, col) in enumerate(JOINTS):
            y = legend.y + 36 + i * 22
            m = self.link.motors[cid]
            pygame.draw.rect(surf, col, (legend.x + 14, y + 7, 20, 4), border_radius=2)
            text(surf, "%s  #%d" % (name, cid), (legend.x + 44, y), 13, BEIGE)
            online = m.ok and time.monotonic() - m.seen < 1.0
            value = ("%+.2f°   %.2f A" % (m.target - m.deg, m.amps)) if online and m.mode else ("off" if online else "offline")
            if m.jam:
                value = "JAMMED   %.2f A" % m.amps
            text(surf, value, (legend.right - 14, y), 13, RED_HI if m.jam else BEIGE if online else DIM, m.jam, "topright")
        top = legend.bottom + 10
        gh = (r.bottom - top - 20) // 3
        specs = [("Tracking error  (target − actual)", "°", 1, True, 1.0),
                 ("Phase current", "A", 2, False, 0.5),
                 ("Temperature", "°C", 3, False, 4.0)]
        for k, spec in enumerate(specs):
            self.graph(surf, pygame.Rect(r.x, top + k * (gh + 10), r.w, gh), *spec)

    def graph(self, surf, r, title, unit, idx, symmetric, min_span):
        pygame.draw.rect(surf, PANEL, r, border_radius=12)
        text(surf, title, (r.x + 14, r.y + 9), 13, BEIGE, True)
        text(surf, unit, (r.right - 14, r.y + 10), 12, MUTED, anchor="topright")
        plot = pygame.Rect(r.x + 50, r.y + 34, r.w - 64, r.h - 56)
        now, win = time.monotonic(), self.cfg["window"]
        series = []
        for _, cid, col in JOINTS:
            segs, seg = [], []
            for h in self.link.motors[cid].hist:
                if h[0] < now - win:
                    continue
                if h[idx] is None:  # motor off: break the line
                    if seg:
                        segs.append(seg)
                    seg = []
                    continue
                seg.append((h[0], h[idx]))
            if seg:
                segs.append(seg)
            series.append((col, segs))
        vals = [v for _, segs in series for seg in segs for _, v in seg]
        pygame.draw.line(surf, LINE, plot.bottomleft, plot.bottomright)
        text(surf, "-%d s" % win, (plot.x, plot.bottom + 4), 11, DIM)
        text(surf, "now", (plot.right, plot.bottom + 4), 11, DIM, anchor="topright")
        if not vals:
            text(surf, "no data yet", plot.center, 12, DIM, anchor="center")
            return
        lo, hi = min(vals), max(vals)
        if symmetric:
            m = max(abs(lo), abs(hi), min_span / 2) * 1.15
            lo, hi = -m, m
        else:
            mid, span = (lo + hi) / 2, max(hi - lo, min_span) * 1.25
            lo, hi = mid - span / 2, mid + span / 2
        span = hi - lo
        fmt = "%.0f" if span >= 20 else "%.1f" if span >= 2 else "%.2f"
        y = lambda v: plot.bottom - (v - lo) / span * plot.h
        for k in range(5):
            v = lo + span * k / 4
            pygame.draw.line(surf, GRID, (plot.x, y(v)), (plot.right, y(v)))
            text(surf, fmt % v, (plot.x - 8, y(v)), 11, DIM, anchor="midright")
        if lo < 0 < hi:
            pygame.draw.line(surf, LINE, (plot.x, y(0)), (plot.right, y(0)))
        surf.set_clip(plot.inflate(0, 4))
        for col, segs in series:
            for seg in segs:
                if len(seg) > 1:
                    pygame.draw.lines(surf, col, False, [(plot.right - (now - t) / win * plot.w, y(v)) for t, v in seg], 2)
        surf.set_clip(None)

    def draw_conn(self, surf):
        r = pygame.Rect(12, TOP + 8, 380, 330)
        self.panel_bg(surf, r)
        text(surf, "Connection", (r.x + 18, r.y + 14), 17, BEIGE, True)
        text(surf, "LightGateway 1.0 over WiFi (UDP)", (r.x + 18, r.y + 38), 12, MUTED)
        text(surf, "Gateway IP", (r.x + 18, r.y + 62), 12, MUTED)
        text(surf, "Port", (r.x + 252, r.y + 62), 12, MUTED)
        self.field(surf, pygame.Rect(r.x + 18, r.y + 80, 222, 34), "host")
        self.field(surf, pygame.Rect(r.x + 252, r.y + 80, 110, 34), "port")
        self.button(surf, pygame.Rect(r.x + 18, r.y + 126, r.w - 36, 38), "Disconnect" if self.connected else "Connect",
                    self.toggle_connect, "plain" if self.connected else "on", 15, True)
        L = self.link
        online = [m for m in L.motors.values() if m.ok and time.monotonic() - m.seen < 1.0]
        gw = self.connected and L.gateway_online()
        rows = [("WiFi gateway", "online" if gw else "offline", GREEN if gw else RED_HI),
                ("Round trip", "%d ms" % L.rtt if gw and L.rtt is not None else "–", BEIGE),
                ("WiFi signal", "%d dBm" % L.rssi if gw and L.rssi else "–", BEIGE),
                ("Arm telemetry", "%d / %d motors · %.0f Hz" % (len(online), len(L.motors), L.rate()) if online else "none",
                 GREEN if online else MUTED),
                ("Bus voltage", "%.1f V" % (sum(m.volts for m in online) / len(online)) if online else "–", BEIGE)]
        for k, (label, value, col) in enumerate(rows):
            y = r.y + 180 + k * 24
            text(surf, label, (r.x + 18, y), 13, MUTED)
            text(surf, value, (r.right - 18, y), 13, col, True, "topright")
        text(surf, "Join the TwinArm WiFi first (192.168.4.1). Simulator: 127.0.0.1", (r.x + 18, r.bottom - 28), 11, DIM)

    def field(self, surf, rect, key):
        focused = self.focus == key
        pygame.draw.rect(surf, BG, rect, border_radius=8)
        pygame.draw.rect(surf, RED_HI if focused else LINE, rect, 1, border_radius=8)
        img = font(15).render(self.fields[key], True, BEIGE)
        surf.blit(img, (rect.x + 10, rect.centery - img.get_height() // 2), (max(0, img.get_width() - rect.w + 24), 0, rect.w - 20, rect.h))
        if focused and int(time.monotonic() * 2) % 2:
            cx = rect.x + 10 + min(img.get_width(), rect.w - 24) + 1
            pygame.draw.line(surf, BEIGE, (cx, rect.y + 8), (cx, rect.bottom - 8), 2)
        self.hits.append((rect, lambda: setattr(self, "focus", key)))

    def draw_settings(self, surf, W):
        rh = 26
        r = pygame.Rect(W - 12 - 440, TOP + 8, 440, 64 + rh * len(ROWS) + 52)
        self.panel_bg(surf, r)
        text(surf, "Settings", (r.x + 18, r.y + 14), 17, BEIGE, True)
        text(surf, "Saved to settings.json  ·  Shift+click steps ×10", (r.x + 18, r.y + 38), 12, MUTED)
        y = r.y + 64
        for row in ROWS:
            label, key = row[0], row[1]
            if key is None:
                text(surf, label.upper(), (r.x + 18, y + 9), 11, RED_HI, True)
                y += rh
                continue
            text(surf, label, (r.x + 18, y + rh // 2), 13, BEIGE, anchor="midleft")
            v = self.cfg[key]
            if len(row) == 2:
                on = v == -1 if key.startswith("sign") else bool(v)
                lab = ("Inverted" if on else "Normal") if key.startswith("sign") else ("On" if on else "Off")
                self.button(surf, pygame.Rect(r.right - 122, y + 2, 104, rh - 4), lab, lambda k=key: self.flip(k),
                            "on" if on else "plain", 12)
            else:
                step, lo, hi = row[2:]
                self.button(surf, pygame.Rect(r.right - 44, y + 2, 26, rh - 4), "+",
                            lambda k=key, st=step, lo=lo, hi=hi: self.bump(k, st, lo, hi), size=15, bold=True)
                text(surf, "%g" % v, (r.right - 44 - 32, y + rh // 2), 13, BEIGE, True, "center")
                self.button(surf, pygame.Rect(r.right - 44 - 64 - 26, y + 2, 26, rh - 4), "−",
                            lambda k=key, st=step, lo=lo, hi=hi: self.bump(k, -st, lo, hi), size=15, bold=True)
            y += rh
        self.button(surf, pygame.Rect(r.x + 18, r.bottom - 42, 150, 28), "Reset defaults", self.reset, size=12)

    def draw_toast(self, surf, content):
        if not self.toast_msg or time.monotonic() - self.toast_at > 3.5:
            return
        img = font(14).render(self.toast_msg, True, BEIGE)
        r = img.get_rect(center=(content.centerx, content.bottom - BAR - 40)).inflate(28, 14)
        pygame.draw.rect(surf, PANEL2, r, border_radius=10)
        pygame.draw.rect(surf, RED_HI, r, 1, border_radius=10)
        surf.blit(img, img.get_rect(center=r.center))

    def update_cursor(self):
        mouse = pygame.mouse.get_pos()
        hand = (self.control and self.tip_px and (self.drag or math.dist(mouse, self.tip_px) < 24)) or \
            any(fn and rect.collidepoint(mouse) for rect, fn in self.hits)
        want = pygame.SYSTEM_CURSOR_HAND if hand else pygame.SYSTEM_CURSOR_ARROW
        if want != self.cursor:
            self.cursor = want
            try:
                pygame.mouse.set_cursor(want)
            except pygame.error:  # no cursor support, e.g. headless
                pass


def main():
    pygame.init()
    icon = pygame.Surface((32, 32), pygame.SRCALPHA)
    pygame.draw.rect(icon, RED, icon.get_rect(), border_radius=8)
    icon_remote(icon, (16, 14), BEIGE)
    pygame.display.set_icon(icon)
    pygame.display.set_caption("TwinArm · Wireless Interface")
    pygame.display.set_mode((1440, 880), pygame.RESIZABLE)
    app = App(sys.argv[1] if len(sys.argv) > 1 else None)
    clock = pygame.time.Clock()
    while app.running:
        for e in pygame.event.get():
            app.event(e)
        app.update()
        app.draw(pygame.display.get_surface())
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
