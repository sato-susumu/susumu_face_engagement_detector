"""Generate a Markdown comparison report from baseline JSON files.

Reads any number of JSON files and groups them by `dataset` field. Detection
runs (with `overall.ap`) and recognition runs (with `accuracy.accuracy`)
render in separate sections.

Usage:
    python3 -m eval.reports.generate_report \
        eval/baselines/*.json \
        --output eval/reports/REPORT.md
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List


def _load_results(paths: List[Path]) -> List[dict]:
    runs = []
    for p in sorted(paths):
        try:
            data = json.loads(p.read_text())
        except json.JSONDecodeError as e:
            print(f"  WARNING: skipping unreadable {p}: {e}")
            continue
        data["_path"] = str(p)
        runs.append(data)
    return runs


def _fmt_pct(value: float | None) -> str:
    if value is None:
        return "—"
    if isinstance(value, float) and value != value:  # NaN
        return "n/a"
    return f"{value * 100:.2f}"


def _fmt_lat(lat: dict | None) -> str:
    if not lat:
        return "—"
    mean = lat.get("mean") or lat.get("mean_ms")
    p95 = lat.get("p95") or lat.get("p95_ms")
    if mean is None:
        return "—"
    return f"{mean:.1f} / {p95:.1f}" if p95 is not None else f"{mean:.1f}"


def _render_detection(runs: List[dict]) -> List[str]:
    if not runs:
        return []
    lines = ["## Detection — WIDER FACE", ""]
    lines.append("| Label | Backend | N images | Overall AP@0.5 | Easy | Medium | Hard | Latency mean/p95 (ms) | Source |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---|")
    for r in runs:
        subsets = r.get("subsets") or {}
        overall = r.get("overall") or {}
        lines.append(
            "| {label} | `{backend}` | {n} | {ap} | {easy} | {medium} | {hard} | {lat} | `{src}` |".format(
                label=r.get("label") or r.get("backend", "?"),
                backend=r.get("backend", "?"),
                n=r.get("num_images", 0),
                ap=_fmt_pct(overall.get("ap")),
                easy=_fmt_pct((subsets.get("easy") or {}).get("ap")),
                medium=_fmt_pct((subsets.get("medium") or {}).get("ap")),
                hard=_fmt_pct((subsets.get("hard") or {}).get("ap")),
                lat=_fmt_lat(r.get("latency_ms")),
                src=Path(r["_path"]).name,
            )
        )
    lines.append("")
    return lines


def _render_recognition(runs: List[dict]) -> List[str]:
    if not runs:
        return []
    lines = ["## Recognition — LFW pairs", ""]
    lines.append("| Label | Backend | Subset | Pairs | Accuracy | Threshold | TAR@FAR=0.1 | TAR@FAR=0.01 | Embed latency mean/p95 (ms) | Source |")
    lines.append("|---|---|---|---:|---:|---:|---:|---:|---:|---|")
    for r in runs:
        acc = r.get("accuracy") or {}
        tar = r.get("tar_at_far") or {}
        lines.append(
            "| {label} | `{backend}` | {subset} | {n} | {acc} | {thr} | {t01} | {t001} | {lat} | `{src}` |".format(
                label=r.get("label") or r.get("backend", "?"),
                backend=r.get("backend", "?"),
                subset=r.get("dataset", "?").replace("lfw_pairs_", ""),
                n=r.get("num_pairs_matched", 0),
                acc=_fmt_pct(acc.get("accuracy")),
                thr=f"{acc.get('threshold', 0):.3f}" if acc.get("threshold") is not None else "—",
                t01=_fmt_pct(tar.get("tar@far=0.1")),
                t001=_fmt_pct(tar.get("tar@far=0.01")),
                lat=_fmt_lat(r.get("latency_ms")),
                src=Path(r["_path"]).name,
            )
        )
    lines.append("")
    return lines


def _collect_figures(figures_dir: Path | None, report_path: Path) -> dict:
    """Walk figures_dir and group PNGs by subdir. Returns dict[subdir, list[relpath_from_report]]."""
    if figures_dir is None or not figures_dir.exists():
        return {}
    groups: dict = {}
    for sub in sorted(figures_dir.iterdir()):
        if not sub.is_dir():
            continue
        pngs = sorted(sub.glob("*.png"))
        if not pngs:
            continue
        # Paths relative to where the report will be written.
        try:
            rels = [Path("..") / p.relative_to(report_path.parent.parent) for p in pngs]
        except ValueError:
            # Fallback: absolute paths (less portable but always valid).
            rels = [p.resolve() for p in pngs]
        groups[sub.name] = [str(r).replace("\\", "/") for r in rels]
    return groups


def _render_figures_section(figures: dict) -> List[str]:
    if not figures:
        return []
    lines = ["## Figures", ""]
    section_titles = {
        "detection": "Detection",
        "recognition": "Recognition (LFW)",
        "engagement": "Engagement",
    }
    for key, paths in figures.items():
        title = section_titles.get(key, key.title())
        lines.append(f"### {title}")
        lines.append("")
        for rel in paths:
            name = Path(rel).stem.replace("_", " ")
            lines.append(f"![{name}]({rel})")
            lines.append("")
    return lines


def render(runs: List[dict], figures: dict | None = None) -> str:
    det = [r for r in runs if "overall" in r and isinstance(r["overall"], dict) and "ap" in r["overall"]]
    rec = [r for r in runs if "accuracy" in r and isinstance(r["accuracy"], dict) and "accuracy" in r["accuracy"]]

    lines: List[str] = []
    lines.append("# Evaluation baselines\n")
    lines.append(
        "Detection AP is the PASCAL VOC all-points interpolation at IoU≥0.5. "
        "Recognition accuracy is at the best cosine-similarity threshold over "
        "the listed pair subset. Latency columns show mean / p95 in milliseconds.\n"
    )
    lines.append("")

    lines += _render_detection(det)
    lines += _render_recognition(rec)
    lines += _render_figures_section(figures or {})

    # Per-run details
    lines.append("## Run details")
    lines.append("")
    for r in runs:
        cfg = r.get("config", {}) if isinstance(r.get("config"), dict) else {}
        lines.append(f"### {r.get('label') or r.get('backend')}")
        lines.append("")
        lines.append(f"- Source: `{r['_path']}`")
        lines.append(f"- Backend: `{r.get('backend')}`")
        lines.append(f"- Dataset: `{r.get('dataset')}`")
        if "num_images" in r:
            lines.append(f"- Images: {r['num_images']}")
        if "num_pairs_matched" in r:
            lines.append(f"- Pairs matched: {r['num_pairs_matched']}")
        if r.get("elapsed_seconds"):
            lines.append(f"- Wall time: {r['elapsed_seconds']:.1f}s")
        if cfg:
            lines.append(f"- Config: `{json.dumps(cfg)}`")
        lines.append("")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("baselines", type=Path, nargs="+", help="Baseline JSON files")
    p.add_argument("--output", "-o", type=Path, required=True)
    p.add_argument("--figures-dir", type=Path, default=None,
                   help="Root figures directory whose subdirs (detection/, recognition/, "
                        "engagement/) get linked into the report.")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    runs = _load_results(args.baselines)
    if not runs:
        raise SystemExit("no readable baselines")
    figures = _collect_figures(args.figures_dir, args.output)
    md = render(runs, figures=figures)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(md)
    n_fig = sum(len(v) for v in figures.values()) if figures else 0
    print(f"[generate_report] wrote {args.output} ({len(runs)} runs, {n_fig} figures)")


if __name__ == "__main__":
    main()
