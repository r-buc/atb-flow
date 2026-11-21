import unittest
from pathlib import Path
import sys

# Allow importing tools/profile_valve.py from repository root
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

import profile_valve as pv  # noqa: E402


class TestValveProfilingMath(unittest.TestCase):
    def test_compute_free_run_stats(self) -> None:
        # 5 samples inrush (0..100ms), then 20 samples around 25.0 mA, then 2 tail samples
        trace = [145.0, 110.0, 65.0, 35.0, 28.0] + [25.0, 25.2, 24.8, 25.1, 24.9] * 4 + [30.0, 40.0]
        stats = pv.compute_free_run_stats(trace_ma=trace, sample_ms=25, inrush_ms=100, tail_trim_samples=2)

        self.assertAlmostEqual(stats["i_free_ma"], 25.0, places=2)
        self.assertAlmostEqual(stats["i_stall_open_ma"], 2.2 * 25.0, places=2)
        self.assertAlmostEqual(stats["i_stall_close_ma"], 2.5 * 25.0, places=2)
        # 0.35 * 25.0 = 8.75 mA, which is larger than 4 * sigma (~0.63 mA)
        self.assertAlmostEqual(stats["delta_i_touch_ma"], 8.75, places=2)
        self.assertAlmostEqual(stats["touch_threshold_ma"], 33.75, places=2)

    def test_detect_touch_point_on_synthetic_closing_curve(self) -> None:
        # Simulate a closing stroke at sample_ms = 25:
        # - 0..125 ms (6 samples): inrush spike (130 -> 30 mA)
        # - 150..2975 ms (114 samples): free run (unloaded plunger gap) at ~25.0 mA
        # - 3000..7500 ms (180 samples): plunger hits valve pin at 3000ms (sample 120),
        #   current jumps to ~36 mA and ramps up to 65 mA stall
        inrush = [130.0, 95.0, 60.0, 40.0, 28.0, 25.5]
        free_run = [25.0] * 114  # indices 6..119 (up to 2975 ms)
        working_stroke = [36.0 + 0.16 * i for i in range(180)]  # starts at index 120 (3000 ms)
        trace = inrush + free_run + working_stroke

        res = pv.detect_touch_point(
            trace_ma=trace,
            i_free_ma=25.0,
            delta_i_touch_ma=8.75,  # threshold = 33.75 mA
            sample_ms=25,
            inrush_ms=150,
            window_size=5,
        )
        self.assertIsNotNone(res)
        assert res is not None
        idx, t_touch_ms, avg_ma = res
        # Touch starts at index 120; 5-sample moving average crosses 33.75 mA by index 123 (3075 ms)
        self.assertTrue(120 <= idx <= 125, f"Unexpected touch index {idx}")
        self.assertTrue(3000 <= t_touch_ms <= 3125, f"Unexpected t_touch_ms {t_touch_ms}")
        self.assertGreaterEqual(avg_ma, 33.75)

    def test_compute_calibration_parameters(self) -> None:
        # t_touch = 3000 ms, t_close = 8000 ms -> T_hub_zu = 5000 ms, eta_work = 5000/8000 = 0.625
        # t_open_total = 7600 ms -> T_hub_auf = 0.625 * 7600 = 4750 ms
        cal = pv.compute_calibration_parameters(
            t_touch_ms=3000,
            t_close_ms=8000,
            t_open_total_ms=7600,
        )
        self.assertEqual(cal["t_hub_zu_ms"], 5000.0)
        self.assertAlmostEqual(cal["eta_work"], 0.625, places=4)
        self.assertEqual(cal["t_hub_auf_ms"], 4750.0)

    def test_compute_move_plan_partial_and_endpoints(self) -> None:
        cal = {
            "t_touch_ms": 3000,
            "t_close_ms": 8000,
            "t_open_total_ms": 7600,
            "t_hub_zu_ms": 5000.0,
            "t_hub_auf_ms": 4750.0,
            "i_stall_open_ma": 55.0,
            "i_stall_close_ma": 62.5,
        }

        # Partial closing move: 100% -> 40% (delta = -60% of 5000 ms = 3000 ms)
        plan_close = pv.compute_move_plan(u_old_pct=100.0, u_new_pct=40.0, cal=cal)
        self.assertEqual(plan_close["action"], "DRIVE_TIMED")
        self.assertEqual(plan_close["dir"], "close")
        self.assertEqual(plan_close["max_ms"], 3000)
        self.assertEqual(plan_close["cutoff_ma"], 62.5)
        self.assertFalse(plan_close["full_stall_sync"])

        # Partial opening move: 40% -> 80% (delta = +40% of 4750 ms = 1900 ms)
        plan_open = pv.compute_move_plan(u_old_pct=40.0, u_new_pct=80.0, cal=cal)
        self.assertEqual(plan_open["action"], "DRIVE_TIMED")
        self.assertEqual(plan_open["dir"], "open")
        self.assertEqual(plan_open["max_ms"], 1900)
        self.assertEqual(plan_open["cutoff_ma"], 55.0)
        self.assertFalse(plan_open["full_stall_sync"])

        # Endpoint 0%: must drive to stall for drift elimination
        plan_zero = pv.compute_move_plan(u_old_pct=40.0, u_new_pct=0.0, cal=cal)
        self.assertEqual(plan_zero["action"], "DRIVE_TO_STALL")
        self.assertEqual(plan_zero["dir"], "close")
        self.assertTrue(plan_zero["full_stall_sync"])
        self.assertEqual(plan_zero["target_pct"], 0.0)

    def test_detect_touch_point_knee(self) -> None:
        # 10 samples inrush, 100 samples free-run around 26 mA (including a 33 mA spike),
        # then 150 samples linear spring ramp (+0.35 mA/sample)
        inrush = [50.0, 35.0, 26.0, 25.0, 25.0, 24.0, 33.0, 33.5, 30.0, 26.0]
        free_run = [26.0 + (0.8 if i % 2 == 0 else -0.8) for i in range(100)]
        free_run[50] = 33.8  # outlier must not fool knee regression
        ramp = [26.0 + 0.35 * (i + 1) for i in range(150)]
        trace = inrush + free_run + ramp
        elapsed_ms = len(trace) * 95  # ~95 ms effective sample interval

        knee = pv.detect_touch_point_knee(trace_ma=trace, elapsed_ms=elapsed_ms)
        self.assertIsNotNone(knee)
        assert knee is not None
        # True knee is at index 109 (10 + 100 - 1)
        self.assertTrue(106 <= knee["touch_idx"] <= 113, f"Unexpected knee idx: {knee}")
        self.assertAlmostEqual(knee["i_free_close_ma"], 26.0, delta=1.0)
        self.assertAlmostEqual(knee["ramp_slope_ma_per_sample"], 0.35, delta=0.03)
        self.assertAlmostEqual(knee["t_touch_ms"], knee["touch_idx"] * 95, delta=50)

    def test_live_open_stall_detector_ignores_breakaway_and_catches_stop(self) -> None:
        det = pv.LiveOpenStallDetector()
        # Breakaway from closed seat (decaying 50 -> 20 mA over ~25 samples),
        # then steady ~19 mA free-run, then sharp mechanical stop at 42 / 46 mA
        trace = (
            [12.0, 50.0, 38.0, 41.0, 37.0, 36.0, 35.0, 35.0, 34.0, 37.0]
            + [34.0, 32.0, 31.0, 30.0, 29.0, 28.0, 27.0, 26.0, 25.0, 24.0]
            + [19.0 + (1.0 if i % 2 == 0 else -1.0) for i in range(80)]
            + [29.0, 42.3, 45.2, 51.4]
        )
        hit = None
        for val in trace:
            hit = det.feed(val)
            if hit is not None:
                break
        self.assertIsNotNone(hit)
        assert hit is not None
        self.assertEqual(hit["reason"], "AUTO_STALL_OPEN")
        self.assertEqual(hit["sample_idx"], 102)  # 2nd sample >= threshold (45.2 mA)

    def test_live_open_stall_detector_initial_stall_when_already_at_top(self) -> None:
        det = pv.LiveOpenStallDetector()
        already_top = [15.0, 58.0, 52.0, 49.0, 51.0, 48.0, 50.0, 52.0, 49.0, 51.0, 53.0, 50.0, 52.0, 54.0, 51.0, 53.0]
        hit = None
        for val in already_top:
            hit = det.feed(val)
            if hit is not None:
                break
        self.assertIsNotNone(hit)
        assert hit is not None
        self.assertEqual(hit["reason"], "AUTO_STALL_OPEN_INITIAL")

    def test_live_close_seat_detector_ignores_ramp_and_catches_hard_seat(self) -> None:
        det = pv.LiveCloseSeatDetector()
        inrush = [15.0, 49.0, 25.0, 25.0, 24.0, 22.0, 33.0, 33.5, 30.0, 22.0]
        free_run = [25.5 + (1.5 if i % 3 == 0 else -1.0) for i in range(60)]
        ramp = [26.0 + 0.35 * i for i in range(160)]  # reaches ~81.6 mA linearly
        hard_seat = [96.0, 108.0, 118.0]
        trace = inrush + free_run + ramp + hard_seat

        hit = None
        for val in trace:
            hit = det.feed(val)
            if hit is not None:
                break
        self.assertIsNotNone(hit)
        assert hit is not None
        self.assertEqual(hit["reason"], "AUTO_SEAT_CLOSE")
        # Must trigger on 2nd hard_seat sample (index 10 + 60 + 160 + 1 = 231)
        self.assertEqual(hit["sample_idx"], len(inrush) + len(free_run) + len(ramp) + 1)

    def test_live_close_seat_detector_ignores_touch_point_preload_jump(self) -> None:
        # Real-world trace where valve spring preload causes a sharp jump (~18 -> 48 -> 38 mA)
        # right at Touch-Point (samples 38..45): detector must NOT stop here.
        touch_jump_trace = [
            14.6, 46.2, 22.3, 25.4, 18.0, 17.7, 24.2, 31.9, 33.6, 30.2,
            22.9, 26.0, 23.8, 18.2, 18.7, 21.0, 16.2, 18.4, 15.2, 16.8,
            21.0, 16.0, 15.6, 20.4, 19.3, 15.8, 20.6, 15.1, 15.3, 21.6,
            17.4, 12.0, 16.6, 22.2, 17.6, 15.8, 18.8, 20.2, 29.2, 37.1,
            40.4, 48.3, 42.7, 39.4, 37.3, 38.9,
        ]
        det = pv.LiveCloseSeatDetector()
        for val in touch_jump_trace:
            self.assertIsNone(det.feed(val))

    def test_detect_open_liftoff_knee(self) -> None:
        # 3 samples inrush, 130 samples descending spring unload (40 -> 18 mA),
        # 200 samples free-run plateau (~18 mA), 30 samples upper hump/stop
        inrush = [48.0, 52.0, 42.0]
        unload = [40.0 - (22.0 / 130.0) * i for i in range(130)]
        free_run = [18.0 + (0.6 if i % 2 == 0 else -0.6) for i in range(200)]
        tail = [30.0] * 10 + [16.0] * 10 + [45.0] * 10
        trace = inrush + unload + free_run + tail
        elapsed_ms = len(trace) * 95

        res = pv.detect_open_liftoff_knee(trace_ma=trace, elapsed_ms=elapsed_ms)
        self.assertIsNotNone(res)
        assert res is not None
        # True lift-off at index ~133
        self.assertTrue(128 <= res["liftoff_idx"] <= 138, f"Unexpected liftoff_idx: {res}")
        self.assertAlmostEqual(res["i_free_open_ma"], 18.0, delta=1.0)
        self.assertAlmostEqual(res["t_hub_auf_ms"], res["liftoff_idx"] * 95, delta=100)


if __name__ == "__main__":
    unittest.main()
