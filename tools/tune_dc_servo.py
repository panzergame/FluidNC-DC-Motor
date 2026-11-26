#!/usr/bin/env python3
"""Tune a FluidNC dc_servo axis over serial from its report_ms telemetry.

Phases:
  1. Feedforward: constant-speed moves at several feeds; the steady drive the
     integral settles to is the drive the motor really needs, so a line fit of
     drive against velocity gives kff (slope) and kfs (intercept).
  2. Feedback: coordinate search over velocity_pid ki/kp and position_kp, each
     candidate scored on a fixed move sequence by tracking error and drive
     chatter, relative to the starting gains.
  3. Optionally velocity_filter_hz and setpoint_filter_hz, the same way,
     followed by a fine feedback pass since the filters move its optimum.
  4. Optionally position_deadband: idle_disable folds whatever error is left
     at disable time into the offset, so the band is the largest error a move
     may leave behind.  Picks the smallest band the axis actually comes to
     rest inside of, instead of holding drive against stiction.  The board's
     overshoot_counts is held fixed during the sweep; the parked band reaches
     position_deadband past the overshoot point, so the band need not clear it.
     Band 0 is skipped when overshoot_counts is set, since it disables the
     overshoot too.

Gains are changed live with $ settings and are not saved; the tuned values are
left applied (or the originals restored on failure / Ctrl-C).

Usage:
    tune_dc_servo.py -p /dev/ttyUSB0 --axis x --home --center 100 --span 40
    tune_dc_servo.py -p /dev/ttyUSB0 --axis y --span 40 --filter --deadband
    tune_dc_servo.py -p /dev/ttyUSB0 --axis y --center 100 --park 0 --start kp=0.01 ki=0.6
"""
import argparse
import json
import math
import re
import statistics
import sys
import time

import serial

# Prime, so the snapshot does not phase-lock to ripple at a multiple of the
# 1 ms control period; at 20 ms the sampled vel read ~30% low in cruise.
REPORT_MS = 17
SETTLE_S = 0.25  # idle_ms disables the motor shortly after a move ends
REPORT_RE = re.compile(
    r"\[MSG:INFO: ([A-Z])(?: Axis)? sp:(\S+) pos:(\S+) err_max:(\S+) vel:(\S+)/(\S+) drive:(\S+)%\]"
)
CTRL_X = b"\x18"


class TuneError(Exception):
    pass


class Tripped(Exception):
    pass


def log(msg):
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()


class Board:
    def __init__(self, ser, axis):
        self.ser = ser
        self.axis = axis.upper()
        self.buf = bytearray()
        self.responses = []
        self.values = {}
        self.capture = None
        self.moving = False
        self.alarm = False

    def pump(self, duration=0.0):
        deadline = time.monotonic() + duration
        while True:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buf += data
                while b"\n" in self.buf:
                    line, _, rest = self.buf.partition(b"\n")
                    self.buf = bytearray(rest)
                    self.handle(line.decode(errors="replace").strip())
            if time.monotonic() >= deadline:
                return

    def handle(self, line):
        if not line:
            return
        m = REPORT_RE.match(line)
        if m:
            if m.group(1) == self.axis and self.capture is not None:
                sp, pos, err, vel, vcmd, drive = (float(x) for x in m.groups()[1:])
                self.capture.append(
                    dict(sp=sp, pos=pos, err=err, vel=vel, vcmd=vcmd, drive=drive, moving=self.moving)
                )
            return
        if "ALARM" in line:
            self.alarm = True
            log(f"< {line}")
        elif line == "ok" or line.startswith("error"):
            self.responses.append(line)
        elif line.startswith("$") and "=" in line:
            key, _, val = line.partition("=")
            self.values[key] = val
        elif "MSG:ERR" in line or "MSG:WARN" in line:
            log(f"< {line}")

    def command(self, cmd, timeout=5.0):
        self.responses.clear()
        self.ser.write(f"{cmd}\n".encode())
        deadline = time.monotonic() + timeout
        alarm_deadline = None
        while time.monotonic() < deadline:
            self.pump(0.02)
            if self.responses:
                return self.responses[0] == "ok"
            # An alarm can swallow the reply to a pending command.
            if self.alarm:
                alarm_deadline = alarm_deadline or time.monotonic() + 1.0
                if time.monotonic() > alarm_deadline:
                    return False
        raise TuneError(f"timed out waiting for reply to {cmd!r}")

    def sync(self, max_wait=15.0):
        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            self.responses.clear()
            self.ser.write(b"\n")
            self.pump(0.5)
            if "ok" in self.responses:
                return
        raise TuneError("board did not answer")

    def get(self, path):
        key = f"$/{path}"
        self.values.pop(key, None)
        if not self.command(key):
            raise TuneError(f"cannot read {key}")
        self.pump(0.05)
        if key not in self.values:
            raise TuneError(f"no value for {key}")
        return float(self.values[key])

    def set(self, path, value):
        if not self.command(f"$/{path}={value:.6g}"):
            raise TuneError(f"cannot set {path}={value}")

    def reset_alarm(self):
        self.ser.write(CTRL_X)
        self.pump(2.0)
        self.sync()
        self.alarm = False
        self.command("$X")
        self.pump(0.2)
        self.alarm = False

    def wait_idle(self, timeout):
        # G4 waits for the planner to drain, so its ok marks the end of motion.
        return self.command("G4 P0.01", timeout=timeout)

    def goto(self, pos, timeout):
        self.command("G90")
        self.command(f"G0 {self.axis}{pos:.3f}")
        return self.wait_idle(timeout)

    def move(self, dist, feed):
        """Runs one relative move and returns its samples, moving then settling."""
        self.capture = []
        self.moving = True
        expected = abs(dist) / feed * 60.0
        ok = self.command(f"G1 {self.axis}{dist:.3f} F{feed:.0f}")
        ok = ok and self.wait_idle(expected + 10.0)
        self.moving = False
        self.pump(SETTLE_S)
        samples, self.capture = self.capture, None
        if self.alarm or not ok:
            raise Tripped()
        return samples


class Tuner:
    PARAMS = {
        "kff": "velocity_pid/kff",
        "kfs": "velocity_pid/kfs",
        "kp": "velocity_pid/kp",
        "ki": "velocity_pid/ki",
        "position_kp": "position_kp",
        "velocity_filter_hz": "velocity_filter_hz",
        "setpoint_filter_hz": "setpoint_filter_hz",
        "position_deadband": "position_deadband",
        "overshoot_counts": "overshoot_counts",
    }

    def __init__(self, board, args):
        self.b = board
        self.args = args
        self.base = f"axes/{args.axis}/motor0/dc_servo"
        self.params = {k: f"{self.base}/{p}" for k, p in self.PARAMS.items()}
        self.original = {}
        self.good = {}
        self.history = []

    def read_all(self):
        vals = {k: self.b.get(p) for k, p in self.params.items()}
        vals["report_ms"] = self.b.get(f"{self.base}/report_ms")
        return vals

    def apply(self, gains):
        for k, v in gains.items():
            if k in self.params:
                self.b.set(self.params[k], v)

    def prepare_motion(self):
        self.b.command("G21")
        if self.args.home:
            log(f"> $H{self.b.axis}")
            if not self.b.command(f"$H{self.b.axis}", timeout=120.0):
                raise TuneError("homing failed")
        if self.args.center is not None:
            self.b.goto(self.args.center, timeout=60.0)
        self.b.command("G91")

    def recover(self):
        log("  tripped: resetting, restoring last good gains")
        self.b.reset_alarm()
        self.apply(self.good)
        self.b.set(f"{self.base}/report_ms", REPORT_MS)
        self.prepare_motion()

    # Phase 1 -----------------------------------------------------------------

    @staticmethod
    def cruise(samples, target_vel):
        """Samples well into the constant-speed part of a move."""
        out, run = [], 0
        moving = [s for s in samples if s["moving"]]
        # At low feeds the setpoint moves a fractional number of counts per
        # report, so a single-report difference is too quantized to compare.
        window = 10
        span_s = window * REPORT_MS / 1000.0
        for prev, cur in zip(moving, moving[window:]):
            spv = (cur["sp"] - prev["sp"]) / span_s
            if abs(spv - target_vel) <= 0.05 * abs(target_vel):
                run += 1
                # Skip the transient left over from the acceleration.
                if run > 10:
                    out.append(dict(cur, pvel=(cur["pos"] - prev["pos"]) / span_s))
            else:
                run = 0
        return out

    def counts_per_s(self, feed):
        return feed / 60.0 * self.steps_per_mm * self.counts_per_step

    def identify_feedforward(self):
        log("Phase 1: feedforward identification")
        points = {+1: [], -1: []}
        for frac in (0.1, 0.25, 0.5, 0.75, 0.95):
            feed = self.max_rate * frac
            v = feed / 60.0
            dist = max(8.0, min(self.args.span, v * 1.5 + v * v / self.accel))
            for sign in (+1, -1):
                samples = self.b.move(sign * dist, feed)
                cr = self.cruise(samples, sign * self.counts_per_s(feed))
                if len(cr) < 5:
                    log(f"  F{feed:.0f} dir {sign:+d}: too few cruise samples ({len(cr)})")
                    continue
                vel = statistics.fmean(abs(s["pvel"]) for s in cr)
                snap = statistics.fmean(abs(s["vel"]) for s in cr)
                drive = statistics.fmean(abs(s["drive"]) for s in cr)
                points[sign].append((vel, drive))
                log(f"  F{feed:6.0f} dir {sign:+d}: vel {vel:7.1f} (reported {snap:7.1f}) counts/s  drive {drive:5.1f}%")
        fits = {}
        for sign, pts in points.items():
            if len(pts) < 3:
                raise TuneError("not enough cruise data for the feedforward fit")
            fits[sign] = statistics.linear_regression([p[0] for p in pts], [p[1] for p in pts])
            log(f"  dir {sign:+d}: drive = {fits[sign].intercept:.2f} + {fits[sign].slope:.5f} * vel")
        kff = statistics.fmean(f.slope for f in fits.values())
        intercept = statistics.fmean(f.intercept for f in fits.values())
        # Friction varies along the travel; overestimating it pushes the motor
        # past a slow setpoint, which the loop then has to pull back.
        kfs = max(0.0, 0.8 * intercept)
        return dict(kff=max(0.0, kff), kfs=min(kfs, 100.0)), points

    # Phase 2 -----------------------------------------------------------------

    def run_sequence(self):
        span, slow = self.args.span, self.args.slow_span
        moves = [
            (+span, self.max_rate),
            (-span, self.max_rate),
            (+slow, self.max_rate * 0.1),
            (-slow, self.max_rate * 0.1),
        ]
        errs, stds, osc = [], [], 0
        for dist, feed in moves:
            samples = self.b.move(dist, feed)
            errs += [s["err"] for s in samples if s["moving"]]
            cr = self.cruise(samples, math.copysign(self.counts_per_s(feed), dist))
            if len(cr) >= 5:
                stds.append(statistics.pstdev(s["drive"] for s in cr))
            settle = [s["drive"] for s in samples if not s["moving"] and abs(s["drive"]) > 2.0]
            osc = max(osc, sum(1 for a, b in zip(settle, settle[1:]) if (a > 0) != (b > 0)))
        return dict(
            err_rms=math.sqrt(statistics.fmean(e * e for e in errs)) if errs else float("inf"),
            err_peak=max(errs, default=float("inf")),
            drive_std=statistics.fmean(stds) if stds else float("inf"),
            osc=osc,
        )

    def score(self, m):
        return m["err_rms"] / self.norm[0] + m["drive_std"] / self.norm[1]

    def evaluate(self, gains, label):
        self.apply(gains)
        try:
            m = self.run_sequence()
        except Tripped:
            self.recover()
            m = dict(err_rms=float("inf"), err_peak=float("inf"), drive_std=float("inf"), osc=0, tripped=True)
        rejected = m.get("tripped") or m["osc"] >= 4 or m["err_peak"] > 0.6 * self.max_following_error
        if self.norm is None or rejected:
            score = float("inf") if rejected else None
        else:
            score = self.score(m)
        m.update(score=score, gains=dict(gains), label=label)
        self.history.append(m)
        s = "rejected" if rejected else (f"score {score:.3f}" if score is not None else "")
        log(
            f"  {label:28s} err_rms {m['err_rms']:7.1f}  peak {m['err_peak']:6.0f}  "
            f"drive_std {m['drive_std']:5.2f}  osc {m['osc']}  {s}"
        )
        return m

    def best_of(self, candidates):
        """Evaluates (gains, label) pairs; returns the best result, or None if there were none."""
        best = None
        for gains, label in candidates:
            m = self.evaluate(gains, label)
            if best is None or m["score"] < best["score"]:
                best = m
        return best

    def search(self, current, keys, passes):
        cur_score = self.evaluate(current, "current")["score"]
        for mults in passes:
            for key in keys:
                if current[key] == 0:
                    continue
                best = self.best_of(
                    (dict(current, **{key: current[key] * mult}), f"{key}={current[key] * mult:.5g} (x{mult})")
                    for mult in mults
                )
                self.apply(current)
                if best["score"] < cur_score * (1.0 - self.margin):
                    current, cur_score = best["gains"], best["score"]
                    self.good = dict(current)
                    log(f"  -> {key} = {current[key]:.5g}")
                else:
                    log(f"  -> {key} unchanged")
        return current, cur_score

    # Phase 4 -----------------------------------------------------------------

    def rest_sequence(self):
        """Per move: |error| and drive at the last report before idle disable."""
        out = []
        for dist in (0.5, 2.0, 10.0):
            for feed in (self.max_rate * 0.1, self.max_rate):
                for sign in (+1, -1):
                    samples = self.b.move(sign * dist, feed)
                    rest = [s for s in samples if not s["moving"]]
                    if rest:
                        last = rest[-1]
                        out.append((abs(last["sp"] - last["pos"]), abs(last["drive"])))
        return out

    def tune_deadband(self, gains):
        log(f"Phase 4: position_deadband (overshoot_counts {gains['overshoot_counts']:.0f})")
        results = {}
        bands = (0, 5, 10, 20, 35, 50)
        if gains["overshoot_counts"]:
            bands = bands[1:]
        for band in bands:
            cand = dict(gains, position_deadband=band)
            self.apply(cand)
            try:
                rest = self.rest_sequence()
            except Tripped:
                self.recover()
                log(f"  deadband {band:3d}: tripped")
                continue
            errs = sorted(e for e, _ in rest)
            held = sum(1 for _, d in rest if d > 0.0)
            p95 = errs[min(len(errs) - 1, int(0.95 * len(errs)))]
            results[band] = dict(err_mean=statistics.fmean(errs), err_p95=p95, held=held, n=len(rest))
            log(
                f"  deadband {band:3d}: rest err mean {results[band]['err_mean']:5.1f}  p95 {p95:5.1f}  "
                f"still driving {held}/{len(rest)}"
            )
        self.history.append(dict(label="deadband", results=results))
        # A move ending outside the band keeps driving until idle disable, so
        # the band should cover the stiction residual of nearly every move.
        ok = [b for b, r in results.items() if r["held"] <= 0.1 * r["n"]]
        band = min(ok) if ok else max(results, default=gains["position_deadband"])
        gains = dict(gains, position_deadband=band)
        self.good = dict(gains)
        self.apply(gains)
        log(f"  -> position_deadband = {band}")
        return gains

    # -------------------------------------------------------------------------

    def run(self):
        b = self.b
        self.max_rate = b.get(f"axes/{self.args.axis}/max_rate_mm_per_min")
        self.accel = b.get(f"axes/{self.args.axis}/acceleration_mm_per_sec2")
        self.steps_per_mm = b.get(f"axes/{self.args.axis}/steps_per_mm")
        self.counts_per_step = b.get(f"{self.base}/encoder/counts_per_step")
        self.max_following_error = b.get(f"{self.base}/max_following_error") or float("inf")
        self.original = self.read_all()
        self.good = {k: v for k, v in self.original.items() if k in self.params}
        log(f"Original: {json.dumps(self.original)}")
        if self.args.start:
            # $ reports round to three decimals, so a starting point taken
            # from another axis is given here rather than read back.
            self.good.update(self.args.start)
            self.apply(self.good)
            log(f"Start: {json.dumps(self.good)}")

        b.set(f"{self.base}/report_ms", REPORT_MS)
        self.prepare_motion()

        gains = dict(self.good)
        self.norm = None
        ff, ff_points = self.identify_feedforward()
        log(f"  fitted kff {ff['kff']:.5f}  kfs {ff['kfs']:.2f}")

        log("Phase 2: baseline")
        base = [self.evaluate(gains, "original") for _ in range(2)]
        if any(m["score"] == float("inf") for m in base):
            raise TuneError("the original gains are rejected by the scoring; tune them by hand first")
        self.norm = (
            max(statistics.fmean(m["err_rms"] for m in base), 1.0),
            max(statistics.fmean(m["drive_std"] for m in base), 0.1),
        )
        scores = [self.score(m) for m in base]
        self.margin = max(0.03, abs(scores[0] - scores[1]) / statistics.fmean(scores))
        cur_score = statistics.fmean(scores)
        log(f"  baseline score {cur_score:.3f}, noise margin {self.margin:.1%}")

        cand = dict(gains, **ff)
        m = self.evaluate(cand, "fitted feedforward")
        if m["score"] < cur_score * (1.0 + self.margin):
            gains, self.good = cand, dict(cand)
            log("  -> fitted feedforward kept")
        else:
            self.apply(gains)
            log("  -> fitted feedforward rejected, keeping the original")

        log("Phase 2: feedback search")
        gains, cur_score = self.search(
            gains, ["ki", "kp", "position_kp"], [(0.5, 0.7, 1.4, 2.0), (0.8, 1.25)]
        )

        if self.args.filter:
            # Edge timing leaves the measured velocity free of count
            # quantization, and counts_per_step < 1 shrinks the setpoint's, so
            # both can sit well above the old count-differencing cutoffs.
            for key, candidates in (
                ("velocity_filter_hz", (50, 100, 200, 400)),
                ("setpoint_filter_hz", (25, 50, 100, 200)),
            ):
                log(f"Phase 3: {key}")
                best = self.best_of((dict(gains, **{key: hz}), f"{key}={hz}") for hz in candidates if hz != gains[key])
                if best is not None and best["score"] < cur_score * (1.0 - self.margin):
                    gains, cur_score = best["gains"], best["score"]
                    self.good = dict(gains)
                self.apply(gains)
                log(f"  -> {key} = {gains[key]:g}")
            log("Phase 3: fine feedback pass")
            gains, cur_score = self.search(gains, ["ki", "kp", "position_kp"], [(0.8, 1.25)])

        if self.args.deadband:
            gains = self.tune_deadband(gains)

        b.set(f"{self.base}/report_ms", self.original["report_ms"])
        return dict(original=self.original, tuned=gains, score=cur_score, ff_points=ff_points, history=self.history)

    def park(self):
        """Without homing, opening the port resets the board and makes wherever
        the axis stands its new zero, so it has to be left where that is safe."""
        if self.args.park is None:
            return
        log(f"> park at {self.b.axis}{self.args.park:g}")
        if not self.b.goto(self.args.park, timeout=120.0):
            log("  park failed")

    def restore(self):
        try:
            self.b.reset_alarm()
            self.apply(self.original)
            if "report_ms" in self.original:
                self.b.set(f"{self.base}/report_ms", self.original["report_ms"])
            log("Original gains restored")
        except Exception as e:  # noqa: BLE001 - best effort while already failing
            log(f"could not restore the original gains: {e}")


def yaml_snippet(axis, g):
    return (
        f"  {axis}:\n    motor0:\n      dc_servo:\n"
        f"        position_kp: {g['position_kp']:.4g}\n"
        f"        velocity_filter_hz: {g['velocity_filter_hz']:.4g}\n"
        f"        setpoint_filter_hz: {g['setpoint_filter_hz']:.4g}\n"
        f"        position_deadband: {g['position_deadband']:.0f}\n"
        f"        overshoot_counts: {g['overshoot_counts']:.0f}\n"
        f"        velocity_pid:\n"
        f"          kp: {g['kp']:.4g}\n          ki: {g['ki']:.4g}\n"
        f"          kff: {g['kff']:.4g}\n          kfs: {g['kfs']:.4g}\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-p", "--port", required=True)
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--axis", required=True, choices=list("xyzabc"))
    parser.add_argument("--home", action="store_true", help="home the axis first (and after a trip)")
    parser.add_argument("--center", type=float, help="absolute position to run the tests around")
    parser.add_argument("--span", type=float, default=40.0, help="length of the fast test moves, mm")
    parser.add_argument("--slow-span", type=float, default=6.0, help="length of the slow test moves, mm")
    parser.add_argument("--filter", action="store_true", help="also tune velocity_filter_hz and setpoint_filter_hz")
    parser.add_argument("--deadband", action="store_true", help="also tune position_deadband")
    parser.add_argument("--park", type=float, help="absolute position to return to at the end, also on failure")
    parser.add_argument(
        "--start", nargs="+", metavar="KEY=VALUE", default=[],
        help="starting values instead of the board's, e.g. kp=0.01 ki=0.6 position_deadband=5",
    )
    parser.add_argument("--json", help="write the full results to this file")
    args = parser.parse_args()
    start = {}
    for item in args.start:
        key, sep, val = item.partition("=")
        if not sep or key not in Tuner.PARAMS:
            parser.error(f"--start {item!r}: expected KEY=VALUE with KEY in {', '.join(Tuner.PARAMS)}")
        start[key] = float(val)
    args.start = start

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        board = Board(ser, args.axis)
        tuner = Tuner(board, args)
        try:
            board.sync()
            board.command("$X")
            result = tuner.run()
        except (TuneError, Tripped, KeyboardInterrupt, serial.SerialException) as e:
            log(f"\nerror: {e!r}")
            tuner.restore()
            try:
                tuner.park()
            except Exception as pe:  # noqa: BLE001 - best effort while already failing
                log(f"could not park: {pe}")
            return 1
        tuner.park()
        if args.json:
            with open(args.json, "w") as f:
                json.dump(result, f, indent=2)
        print(yaml_snippet(args.axis, result["tuned"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
