"""Command-line entry point for explicitly labeled mock replay reports."""
import argparse
import hashlib
import json
from pathlib import Path
from .replay import Policy, run_scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/replay-report.json"))
    parser.add_argument("--max-retries", type=int, default=1)
    parser.add_argument("--max-frame-age-ms", type=float, default=500)
    args = parser.parse_args()
    try:
        raw = args.scenario.read_bytes()
        report = run_scenario(json.loads(raw), Policy(args.max_retries, args.max_frame_age_ms))
        report["input_sha256"] = hashlib.sha256(raw).hexdigest()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))
    print(f"MOCK REPLAY ONLY: {report['metrics']['tasks_completed']}/{report['metrics']['tasks_started']} fixture tasks completed")
    print(f"Report: {args.output}")


if __name__ == "__main__":
    main()
