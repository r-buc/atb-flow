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


if __name__ == "__main__":
    unittest.main()
