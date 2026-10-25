import random
import statistics as st

from handoff_core import sim

MODES = ["naive (no margin)", "fixed margin 0.3 s", "adaptive rule"]
DELAYS = [0.0, 0.1, 0.3, 0.6]   # one-way base delay, seconds
N = 200                         # trials per condition
SPIKE = 0.05                    # probability a message gets an extra delay spike


def margin_for(mode, p1, p2):
    if mode.startswith("naive"):
        return 0.0
    if mode.startswith("fixed"):
        return 0.3
    return sim.rule_margin(p1, p2)


print("nominal timeline:", sim.NOM)
print(f"{'delay':>6} {'mode':<20} {'guard':<5} {'collide%':>8} {'stall%':>7} {'handoff s':>9}")
for d in DELAYS:
    jit = 0.3 * d
    for mode in MODES:
        for guard in (False, True):
            coll = stall = 0
            times = []
            for i in range(N):
                rng = random.Random(10_000 + i)
                l1 = sim.Link(d, jit, SPIKE, rng)
                l2 = sim.Link(d, jit, SPIKE, rng)
                plan = sim.schedule(margin_for(mode, sim.probe(l1), sim.probe(l2)))
                r = sim.run_trial(plan, d, jit, SPIKE, guard, seed=i)
                coll += r["conflict"]
                stall += (not r["finished"])
                times.append(r["handoff_time"])
            print(f"{d*1000:>5.0f}ms {mode:<20} {str(guard):<5} {100*coll/N:>8.1f} {100*stall/N:>7.1f} {st.mean(times):>9.2f}")
