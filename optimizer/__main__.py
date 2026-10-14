"""Start the overnight loop.

Webots runs on Windows. ``--dry-run`` exercises diagnose / pick / score /
logging without a simulator and writes a synthetic morning report.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from optimizer.loop import run_loop
from optimizer.picker import LlmPicker, RulePicker
from optimizer.runner import REPO_ROOT, DryRunRunner, WebotsLapRunner


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Overnight Webots optimizer for energy-model accuracy and drive Wh/m."
    )
    parser.add_argument("--max-iterations", type=int, default=10)
    parser.add_argument("--max-hours", type=float, default=8.0)
    parser.add_argument("--lap-timeout-min", type=float, default=40.0)
    parser.add_argument("--world", default="warehouse")
    parser.add_argument("--picker", choices=("rules", "llm"), default="rules")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "config" / "config.yaml")
    parser.add_argument("--knowledge", type=Path, default=REPO_ROOT / "optimizer" / "KNOWLEDGE.md")
    parser.add_argument("--run-dir", type=Path, default=REPO_ROOT / "optimizer" / "runs")
    parser.add_argument(
        "--initial-log",
        type=Path,
        default=None,
        help="Use this text as the last mission instead of docker logs.",
    )
    args = parser.parse_args(argv)
    if args.max_iterations < 0 or args.max_hours <= 0 or args.lap_timeout_min <= 0:
        print("Limits must be positive.", file=sys.stderr)
        return 2
    if not args.dry_run and os.name != "nt":
        print(
            "This optimizer launches Webots on Windows. On this machine pass --dry-run "
            "to check the report, or start scripts\\overnight_optimize.ps1 on the PC.",
            file=sys.stderr,
        )
        return 2
    if args.picker == "llm":
        picker = LlmPicker(os.environ.get("OPTIMIZER_LLM_CMD", ""))
    else:
        picker = RulePicker()
    initial = None
    if args.initial_log is not None:
        initial = args.initial_log.read_text(encoding="utf-8")
    if args.dry_run:
        runner = DryRunRunner()
        fetch = None
    else:
        runner = WebotsLapRunner(REPO_ROOT, world=args.world)
        fetch = None if initial is not None else runner.recent_log
    report = run_loop(
        config_path=args.config,
        knowledge_path=args.knowledge,
        run_dir=args.run_dir,
        runner=runner,
        picker=picker,
        max_iterations=args.max_iterations,
        max_hours=args.max_hours,
        lap_timeout_s=args.lap_timeout_min * 60.0,
        world=args.world,
        initial_log=initial,
        fetch_existing=fetch,
        dry_run=args.dry_run,
    )
    print(f"Morning report: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
