"""Visualisation entry point: produces PNG figures under outputs/figures/.

Subcommands:
    detection    overlays + AP/latency bar charts
    recognition  cosine-similarity distributions + ROC
    engagement   synthetic engagement-score timeline

Each subcommand is self-contained — pick what you need or run them all via
``make visualize``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List

import matplotlib

matplotlib.use("Agg")  # headless backend — safe in CI / SSH sessions
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _load_baselines(baselines_dir: Path) -> List[dict]:
    runs = []
    for p in sorted(baselines_dir.glob("*.json")):
        try:
            data = json.loads(p.read_text())
            data["_path"] = str(p)
            data["_name"] = p.stem
            runs.append(data)
        except json.JSONDecodeError as e:
            print(f"  warn: skip unreadable {p}: {e}", file=sys.stderr)
    return runs


def _detection_runs(runs: List[dict]) -> List[dict]:
    return [r for r in runs if isinstance(r.get("overall"), dict) and "ap" in r["overall"]]


def _recognition_runs(runs: List[dict]) -> List[dict]:
    return [r for r in runs if isinstance(r.get("accuracy"), dict) and "accuracy" in r["accuracy"]]


# ---------------------------------------------------------------------------
# Detection: overlay images + bar charts
# ---------------------------------------------------------------------------


def _detection_overlay(
    image_path: Path,
    backend_outputs: dict,  # label -> list[(box_tuple, score)]
    out_path: Path,
) -> None:
    import cv2

    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(image_path)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    n = len(backend_outputs)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 5))
    if n == 1:
        axes = [axes]

    for ax, (label, boxes) in zip(axes, backend_outputs.items()):
        ax.imshow(rgb)
        ax.set_title(f"{label}  ({len(boxes)} face{'s' if len(boxes) != 1 else ''})")
        ax.axis("off")
        for (top, right, bottom, left, score) in boxes:
            rect = plt.Rectangle(
                (left, top), right - left, bottom - top,
                fill=False, edgecolor="lime", linewidth=2.0,
            )
            ax.add_patch(rect)
            if score and score < 1.0:
                ax.text(left, max(0, top - 5), f"{score:.2f}",
                        color="lime", fontsize=8,
                        bbox=dict(facecolor="black", alpha=0.5, pad=1))

    fig.suptitle(f"Detection comparison — {image_path.name}", fontsize=12)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")


def _detection_bars(runs: List[dict], out_path: Path) -> None:
    if not runs:
        return
    labels = [r.get("label") or r.get("backend") for r in runs]
    aps = [(r.get("overall") or {}).get("ap", 0.0) * 100 for r in runs]
    lat_mean = [(r.get("latency_ms") or {}).get("mean", 0.0) for r in runs]
    lat_p95 = [(r.get("latency_ms") or {}).get("p95", 0.0) for r in runs]

    fig, (ax_ap, ax_lat) = plt.subplots(1, 2, figsize=(13, 5))

    x = np.arange(len(labels))
    bars = ax_ap.bar(x, aps, color="#4C72B0")
    ax_ap.set_xticks(x)
    ax_ap.set_xticklabels(labels, rotation=15, ha="right", fontsize=9)
    ax_ap.set_ylabel("AP@0.5  (%)")
    ax_ap.set_title("Detection — AP on WIDER FACE val")
    ax_ap.set_ylim(0, max(100, max(aps) * 1.15 if aps else 100))
    ax_ap.grid(True, axis="y", alpha=0.3)
    for b, v in zip(bars, aps):
        ax_ap.text(b.get_x() + b.get_width() / 2, v + 1.5,
                   f"{v:.2f}", ha="center", fontsize=9)

    width = 0.38
    ax_lat.bar(x - width / 2, lat_mean, width, label="mean", color="#55A868")
    ax_lat.bar(x + width / 2, lat_p95, width, label="p95", color="#DD8452")
    ax_lat.set_xticks(x)
    ax_lat.set_xticklabels(labels, rotation=15, ha="right", fontsize=9)
    ax_lat.set_ylabel("Latency per image (ms)")
    ax_lat.set_title("Detection — latency")
    ax_lat.legend()
    ax_lat.grid(True, axis="y", alpha=0.3)
    ax_lat.set_yscale("log")  # huge dynamic range between dlib and YuNet

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")


def cmd_detection(args: argparse.Namespace) -> int:
    baselines_dir = Path(args.baselines_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = _detection_runs(_load_baselines(baselines_dir))
    if not runs:
        print(f"  no detection baselines found under {baselines_dir}", file=sys.stderr)
    else:
        _detection_bars(runs, out_dir / "backend_comparison.png")

    # Per-image overlays — pick a small handful of well-known WIDER FACE val
    # images and run each available backend live.
    dataset_root = Path(args.dataset_root)
    sample_relpaths = [
        "WIDER_val/images/0--Parade/0_Parade_Parade_0_12.jpg",
        "WIDER_val/images/0--Parade/0_Parade_Parade_0_194.jpg",
        "WIDER_val/images/13--Interview/13_Interview_Interview_Sequences_13_50.jpg",
    ]
    sample_images = [p for p in (dataset_root / rp for rp in sample_relpaths) if p.exists()]
    if not sample_images:
        print(f"  no sample WIDER images under {dataset_root} — skipping overlays")
        return 0

    # Lazy-import backends + skip ones whose weights are missing.
    from susumu_face_engagement_detector.backends.detection import make_backend

    backend_specs: list = [("dlib_hog (v0)", "dlib_hog", {})]
    yunet_model = Path(args.yunet_model) if args.yunet_model else None
    if yunet_model and yunet_model.exists():
        backend_specs.append(("YuNet (v1)", "yunet", {"model_path": str(yunet_model)}))

    backends = []
    for label, name, kwargs in backend_specs:
        try:
            backends.append((label, make_backend(name, **kwargs)))
        except Exception as e:
            print(f"  skip backend {name}: {e}")

    import cv2
    for img_path in sample_images:
        bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        backend_outputs: dict = {}
        for label, backend in backends:
            result = backend.detect(bgr)
            backend_outputs[label] = result.boxes
        if not backend_outputs:
            continue
        out_file = out_dir / f"overlay_{img_path.stem}.png"
        _detection_overlay(img_path, backend_outputs, out_file)

    return 0


# ---------------------------------------------------------------------------
# Recognition: cosine distribution + ROC
# ---------------------------------------------------------------------------


def cmd_recognition(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # We need raw pair scores — rerun the embedding on a sample of LFW pairs.
    # Keep it modest (default 200 pairs) so the visualisation step stays fast.
    try:
        from eval.datasets.lfw import load_pairs
        from eval.runners.run_recognition_eval import BACKENDS
        from eval.metrics.recognition import Pair, cosine_distance
    except Exception as e:
        print(f"  cannot load recognition modules: {e}", file=sys.stderr)
        return 1

    print(f"  loading LFW pairs (subset={args.subset})…")
    pairs = load_pairs(subset=args.subset)
    if args.max_pairs and len(pairs) > args.max_pairs:
        # Take a balanced subset.
        same = [p for p in pairs if p.same][: args.max_pairs // 2]
        diff = [p for p in pairs if not p.same][: args.max_pairs // 2]
        pairs = same + diff
    print(f"  using {len(pairs)} pairs ({sum(p.same for p in pairs)} same)")

    backend = BACKENDS[args.backend]()
    print(f"  embedding with {backend.name}…")

    scores_same: List[float] = []
    scores_diff: List[float] = []
    skipped = 0
    for i, lp in enumerate(pairs):
        ea = backend.embed(lp.image_a)
        eb = backend.embed(lp.image_b)
        if ea is None or eb is None:
            skipped += 1
            continue
        sim = 1.0 - cosine_distance(ea, eb)
        (scores_same if lp.same else scores_diff).append(sim)
        if (i + 1) % 50 == 0:
            print(f"    {i + 1}/{len(pairs)} pairs processed", flush=True)
    print(f"  done — {len(scores_same)} same / {len(scores_diff)} different, skipped {skipped}")

    # 1. Distribution histogram
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(scores_diff, bins=40, alpha=0.55, label=f"different person (n={len(scores_diff)})",
            color="#DD8452")
    ax.hist(scores_same, bins=40, alpha=0.55, label=f"same person (n={len(scores_same)})",
            color="#4C72B0")
    ax.set_xlabel("Cosine similarity")
    ax.set_ylabel("Pair count")
    ax.set_title(f"LFW pair score distribution — {backend.name}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    hist_path = out_dir / f"score_distribution_{backend.name}.png"
    fig.savefig(hist_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {hist_path}")

    # 2. ROC curve
    from sklearn.metrics import roc_curve, auc
    y_true = [1] * len(scores_same) + [0] * len(scores_diff)
    y_score = scores_same + scores_diff
    fpr, tpr, _thr = roc_curve(y_true, y_score)
    auc_val = auc(fpr, tpr)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(fpr, tpr, label=f"{backend.name} (AUC={auc_val:.4f})", color="#4C72B0", lw=2)
    ax.plot([0, 1], [0, 1], color="gray", lw=1, linestyle="--", label="chance")
    ax.set_xlabel("False positive rate")
    ax.set_ylabel("True positive rate")
    ax.set_title(f"LFW verification ROC — {backend.name}")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    fig.tight_layout()
    roc_path = out_dir / f"roc_{backend.name}.png"
    fig.savefig(roc_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {roc_path}")
    return 0


# ---------------------------------------------------------------------------
# Engagement: synthetic timeline
# ---------------------------------------------------------------------------


def cmd_engagement(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    from susumu_face_engagement_detector.engagement import (
        EngagementConfig, EngagementInputs, EngagementScorer, state_name,
    )

    cfg = EngagementConfig()
    scorer = EngagementScorer(cfg)

    # Scripted scenario:
    #   0–4s : no detection (UNKNOWN)
    #   4–10s: engaged signals — neutral + camera-aligned gaze
    #   10–14s: distracted — gaze 40° off, yaw 30°
    #   14–20s: re-engaged — neutral again
    #   20–24s: bored — sad emotion, gaze drifting
    ticks_per_sec = 10
    total_s = 24
    times = []
    scores = []
    levels = []
    raw_emotion = []
    raw_gaze = []
    raw_yaw = []

    for tick in range(total_s * ticks_per_sec):
        t = tick / ticks_per_sec
        if t < 4:
            inp = EngagementInputs(None, None, None, None)
        elif t < 10:
            inp = EngagementInputs("neutral", 5.0, 0.0, 0.0)
        elif t < 14:
            inp = EngagementInputs("neutral", 40.0, 30.0, 0.0)
        elif t < 20:
            inp = EngagementInputs("neutral", 5.0, 0.0, 0.0)
        else:
            inp = EngagementInputs("sad", 25.0, 0.0, 0.0)
        st = scorer.update(inp)
        times.append(t)
        scores.append(st.score)
        levels.append(st.level)
        raw_emotion.append(inp.emotion_label or "")
        raw_gaze.append(inp.gaze_angle_deg)
        raw_yaw.append(inp.yaw_deg)

    fig, (ax_score, ax_state, ax_input) = plt.subplots(
        3, 1, figsize=(11, 9), sharex=True,
        gridspec_kw={"height_ratios": [3, 2, 2]},
    )

    # Score line
    ax_score.plot(times, scores, color="#4C72B0", lw=2)
    ax_score.axhline(cfg.engaged_threshold, color="green", linestyle="--",
                     lw=1, label=f"ENGAGED ≥ {cfg.engaged_threshold}")
    ax_score.axhline(cfg.engaging_threshold, color="orange", linestyle="--",
                     lw=1, label=f"ENGAGING ≥ {cfg.engaging_threshold}")
    ax_score.axhline(cfg.disengaged_threshold, color="red", linestyle="--",
                     lw=1, label=f"DISENGAGED < {cfg.disengaged_threshold}")
    ax_score.set_ylabel("Filtered score")
    ax_score.set_ylim(-0.02, 1.05)
    ax_score.set_title("Engagement score over a scripted scenario")
    ax_score.legend(loc="lower right", fontsize=9)
    ax_score.grid(True, alpha=0.3)

    # State bars
    level_colors = {0: "#999999", 1: "#C44E52", 2: "#DD8452",
                    3: "#55A868", 4: "#8172B2"}
    for i, t in enumerate(times):
        dt = 1.0 / ticks_per_sec
        ax_state.barh(0, dt, left=t, color=level_colors[levels[i]], height=0.6)
    ax_state.set_yticks([0])
    ax_state.set_yticklabels(["EngagementLevel"])
    ax_state.set_xlim(0, total_s)
    # Build a legend from unique levels seen
    from matplotlib.patches import Patch
    seen = []
    handles = []
    for lvl in [0, 1, 2, 3, 4]:
        if lvl in levels:
            handles.append(Patch(facecolor=level_colors[lvl], label=state_name(lvl)))
            seen.append(lvl)
    ax_state.legend(handles=handles, loc="upper right", ncol=len(handles), fontsize=9)
    ax_state.set_title("State (5-level EngagementLevel)")

    # Inputs panel — gaze angle + yaw
    ax_input.plot(times, [g if g is not None else np.nan for g in raw_gaze],
                  label="gaze angle from +Z (°)", color="#4C72B0")
    ax_input.plot(times, [y if y is not None else np.nan for y in raw_yaw],
                  label="head yaw (°)", color="#DD8452")
    ax_input.axhline(cfg.gaze_full_deg, color="#4C72B0", linestyle=":", lw=1)
    ax_input.axhline(cfg.yaw_gate_deg, color="#DD8452", linestyle=":", lw=1)
    ax_input.set_xlabel("Time (s)")
    ax_input.set_ylabel("Angle (°)")
    ax_input.set_title("Driving inputs (emotion label shown in subtitle)")
    ax_input.legend(loc="upper right", fontsize=9)
    ax_input.grid(True, alpha=0.3)

    # Phase annotations
    phases = [(0, 4, "UNKNOWN\n(no input)"),
              (4, 10, "engaged signals"),
              (10, 14, "distracted\n(yaw, off-axis gaze)"),
              (14, 20, "re-engaged"),
              (20, 24, "bored\n(sad emotion)")]
    for (a, b, label) in phases:
        ax_score.axvspan(a, b, alpha=0.05, color="black")
        ax_score.text((a + b) / 2, 1.0, label, ha="center", va="bottom",
                      fontsize=8, alpha=0.7)

    fig.tight_layout()
    out_path = out_dir / "engagement_timeline.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("detection", help="Detection overlays + AP/latency bars")
    d.add_argument("--baselines-dir", default="outputs/baselines")
    d.add_argument("--out-dir", default="outputs/figures/detection")
    d.add_argument("--dataset-root", default=str(Path.home() / "datasets" / "wider_face"))
    d.add_argument("--yunet-model",
                   default=str(Path.home() / "models" / "face_detection"
                               / "face_detection_yunet_2023mar.onnx"))
    d.set_defaults(func=cmd_detection)

    r = sub.add_parser("recognition", help="LFW score distribution + ROC")
    r.add_argument("--out-dir", default="outputs/figures/recognition")
    r.add_argument("--backend", default="dlib_128d")
    r.add_argument("--subset", default="test")
    r.add_argument("--max-pairs", type=int, default=200)
    r.set_defaults(func=cmd_recognition)

    e = sub.add_parser("engagement", help="Synthetic engagement-score timeline")
    e.add_argument("--out-dir", default="outputs/figures/engagement")
    e.set_defaults(func=cmd_engagement)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
