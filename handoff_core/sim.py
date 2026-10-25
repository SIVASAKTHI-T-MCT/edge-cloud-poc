"""Pure-Python discrete-time simulation of a two-arm handoff over a delayed network.

Constants match the ROS 2 arm nodes so both describe the same scenario.
"""
import math
import random

ZONE = (0.0, 0.0)
ZONE_R = 0.12
PICK = (-0.95, 0.25)
DROP = (0.95, 0.25)
HOME = {"r1": (-0.45, 0.35), "r2": (0.45, 0.35)}
SPEED = 0.5          # m/s
DT = 0.01            # s
DWELL_PICK = 0.3
DWELL_ZONE = 0.4
GUARD_MARGIN = 0.02  # guard holds this far outside the zone edge
HEARTBEAT = 0.05


def dist_zone(p):
    return math.hypot(p[0] - ZONE[0], p[1] - ZONE[1])


def waypoints(rid):
    """List of (target, dwell_seconds) for one handoff."""
    if rid == "r1":
        return [(PICK, DWELL_PICK), (ZONE, DWELL_ZONE), (HOME["r1"], 0.0)]
    return [(ZONE, DWELL_ZONE), (DROP, DWELL_PICK), (HOME["r2"], 0.0)]


class Arm:
    def __init__(self, rid):
        self.rid = rid
        self.pos = HOME[rid]
        self.plan = []
        self.dwell_left = 0.0
        self.started = False
        self.done_time = None
        self.peer_in_zone = False   # latest peer state received (stale by link delay)
        self.hold_time = 0.0

    def start(self):
        if not self.started:
            self.started = True
            self.plan = list(waypoints(self.rid))

    def in_zone(self):
        return dist_zone(self.pos) <= ZONE_R

    def step(self, t, guard):
        if not self.started or not self.plan:
            return
        target, dwell = self.plan[0]
        if self.dwell_left > 0:
            self.dwell_left -= DT
            if self.dwell_left <= 0:
                self.plan.pop(0)
                if not self.plan:
                    self.done_time = t
            return
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        d = math.hypot(dx, dy)
        move = SPEED * DT
        nxt = target if d <= move else (self.pos[0] + dx / d * move, self.pos[1] + dy / d * move)
        # local guard: do not cross into the zone while the peer was last seen inside it
        if guard and dist_zone(self.pos) > ZONE_R + GUARD_MARGIN and \
                dist_zone(nxt) <= ZONE_R + GUARD_MARGIN and self.peer_in_zone:
            self.hold_time += DT
            return
        self.pos = nxt
        if d <= move:
            if dwell > 0:
                self.dwell_left = dwell
            else:
                self.plan.pop(0)
                if not self.plan:
                    self.done_time = t


def nominal_times():
    """Zone entry/exit times of each arm when run alone with zero network delay."""
    out = {}
    for rid in ("r1", "r2"):
        a = Arm(rid)
        a.start()
        t = 0.0
        enter = exit_ = None
        while a.plan and t < 60:
            a.step(t, guard=False)
            t += DT
            if a.in_zone() and enter is None:
                enter = t
            if enter is not None and exit_ is None and not a.in_zone():
                exit_ = t
        out[rid] = {"enter": enter, "exit": exit_, "done": t}
    return out


NOM = nominal_times()


class Link:
    """One-way link with base delay, uniform jitter and occasional delay spikes."""

    def __init__(self, base, jitter, spike_prob, rng):
        self.base, self.jitter, self.spike_prob, self.rng = base, jitter, spike_prob, rng

    def delay(self):
        d = self.base + self.rng.uniform(-self.jitter, self.jitter)
        if self.rng.random() < self.spike_prob:
            d += self.rng.uniform(0.5, 1.0) * max(self.base, 0.05)
        return max(0.0, d)


def probe(link, n=40):
    xs = sorted(link.delay() for _ in range(n))
    return {"mean": sum(xs) / n, "p95": xs[int(0.95 * (n - 1))], "min": xs[0], "max": xs[-1]}


def schedule(margin):
    """Start offsets so r2 enters `margin` seconds after r1 leaves the zone."""
    s2 = NOM["r1"]["exit"] - NOM["r2"]["enter"] + margin
    return {"start_r1": 0.0, "start_r2": max(0.0, s2), "margin": margin}


def rule_margin(p1, p2, safety=0.1):
    """Margin covering worst-case arrival skew between the two command links."""
    skew = max(p1["p95"] - p2["min"], p2["p95"] - p1["min"], 0.0)
    return skew + safety


def run_trial(plan, delay, jitter, spike_prob, guard, seed, horizon=40.0):
    rng = random.Random(seed)
    cmd1 = Link(delay, jitter, spike_prob, rng)
    cmd2 = Link(delay, jitter, spike_prob, rng)
    peer = Link(delay, jitter, spike_prob, rng)
    r1, r2 = Arm("r1"), Arm("r2")
    arrive1 = plan["start_r1"] + cmd1.delay()
    arrive2 = plan["start_r2"] + cmd2.delay()
    inbox = []          # (deliver_time, arm, in_zone_flag)
    next_hb = 0.0
    t = 0.0
    conflict_time = 0.0
    conflict = False
    while t < horizon:
        if t >= arrive1:
            r1.start()
        if t >= arrive2:
            r2.start()
        if t >= next_hb:
            inbox.append((t + peer.delay(), r2, r1.in_zone()))
            inbox.append((t + peer.delay(), r1, r2.in_zone()))
            next_hb += HEARTBEAT
        inbox.sort(key=lambda m: m[0])
        while inbox and inbox[0][0] <= t:
            _, arm, flag = inbox.pop(0)
            arm.peer_in_zone = flag
        r1.step(t, guard)
        r2.step(t, guard)
        if r1.in_zone() and r2.in_zone():
            conflict = True
            conflict_time += DT
        t += DT
        if r1.done_time is not None and r2.done_time is not None:
            break
    finished = r1.done_time is not None and r2.done_time is not None
    return {
        "conflict": conflict,
        "conflict_time": conflict_time,
        "finished": finished,
        "handoff_time": (r2.done_time if r2.done_time is not None else horizon),
        "hold_time": r1.hold_time + r2.hold_time,
    }
