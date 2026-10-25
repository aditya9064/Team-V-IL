"""Remote-only comparison page; isolated results never become assembly passes."""
import argparse, hashlib, html, json, subprocess, sys
from pathlib import Path

CASES = (("original", "original"), ("stem-plus3", "stem-plus3"))


def build(root):
    root = Path(root)
    source = Path(__file__).parent
    approved = json.loads((source / "PAIRED-GRIP-SOURCE.json").read_text())["runtime_sha256"]
    summary = {"schema": "V55_PAIRED_GEOMETRY_GRIP_REVIEW", "full_assembly_accepted": False,
               "scope": "Only lid pickup and stationary hold; no transport, seating or release",
               "cases": {}, "comparison": "UNVERIFIED_OR_INCOMPLETE"}
    sections = []
    for name, variant in CASES:
        iterations = 1
        folder = root / name
        path = folder / "report.json"
        if not path.is_file():
            summary["cases"][name] = {"status": "NO_REPORT", "iterations": iterations}
            sections.append(f"<section><h2>{name}</h2><p>No report. Not accepted.</p></section>")
            continue
        report = json.loads(path.read_text())
        audit_path = folder / "independent-isolated-audit.json"
        check = subprocess.run([sys.executable, str(Path(__file__).with_name("check_paired_lid_grip.py")),
                                str(path), "--source", str(source), "--expected-runtime-sha256", approved,
                                "--velocity-iterations", str(iterations), "--variant", variant], capture_output=True, text=True)
        (folder / "checker-output.txt").write_text(check.stdout + "\n" + check.stderr)
        audit = json.loads(audit_path.read_text()) if audit_path.is_file() else None
        case = {"iterations": iterations, "geometry_variant": variant, "runtime_status": report.get("status"),
                "runtime_error": report.get("error"), "checker_returncode": check.returncode,
                "audit": audit, "report_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "full_assembly_accepted": False}
        summary["cases"][name] = case
        video = report.get("video", {})
        candidate = video.get("path") or video.get("failed_recording_path")
        media = "<p>No finalized video.</p>"
        if candidate and Path(candidate).is_file():
            actual = Path(candidate).resolve()
            if root.resolve() not in actual.parents:
                raise ValueError("Video must remain within the remote run")
            alias = root / (name + ".mp4")
            if alias.exists() or alias.is_symlink():
                if alias.is_symlink() or alias.stat().st_ino != actual.stat().st_ino:
                    raise ValueError("Video alias already refers to another source")
            else:
                alias.hardlink_to(actual)
            import imageio_ffmpeg
            frames, seconds = imageio_ffmpeg.count_frames_and_secs(str(actual))
            case["video_verified"] = {"frames": frames, "seconds": seconds,
                                      "sha256": hashlib.sha256(actual.read_bytes()).hexdigest(),
                                      "path": str(actual), "stays_remote": True}
            media = f'<video controls preload="metadata" src="{name}.mp4"></video>'
        message = report.get("error") or report.get("status", "UNKNOWN")
        (root / (name + "-audit.json")).write_text(json.dumps(case, indent=2, allow_nan=False) + "\n")
        sections.append(f'<section><h2>{html.escape(name)} — PGS32:{iterations}</h2>'
                        f'<p>{html.escape(message)}</p>{media}'
                        f'<p><a href="{name}-audit.json">Independent checker output</a></p></section>')
    # Mechanical acceptance and comparison interpretation remain separate.
    audits = [c.get("audit") for c in summary["cases"].values()]
    if len(audits) == 2 and all(a and a.get("audit_integrity_valid") for a in audits):
        from check_paired_lid_grip import classify_comparison
        summary["comparison"] = classify_comparison(audits)
    summary["coordinator_raw_evidence_and_visual_audit_pending"] = True
    (root / "review-summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><title>Lid grip diagnostic</title>
<style>body{font:16px system-ui;background:#141414;color:#eee;max-width:1100px;margin:32px auto;padding:0 20px}
section{border-top:1px solid #555;padding:20px 0}video{width:100%;max-height:650px;background:#000}a{color:#ff9b45}
.note{border-left:4px solid #f77c24;padding:12px 18px;background:#242424}</style>
<h1>Original versus 3 mm longer lid gripping post</h1><p class="note">Actual simulator recordings, including failures.
This tests pickup and holding only. It does not show completed assembly. Grippers and strict acceptance limits are unchanged.</p>
''' + "\n".join(sections) + '<p><a href="review-summary.json">Comparison evidence</a></p></html>'
    (root / "review.html").write_text(page)
    print("ISOLATED_GRIP_REVIEW_READY", root, flush=True)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    build(parser.parse_args().run)
