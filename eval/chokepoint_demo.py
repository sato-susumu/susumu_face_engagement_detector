"""Create a ground-truth face-ID demo video from the ChokePoint dataset.

ChokePoint stores video face tracks as 96x96 PGM crops grouped by person ID.
This script downloads a small archive, builds a tiled multi-person video, and
emits a JSON file containing per-frame ground-truth IDs and recognition output.
"""
from __future__ import annotations

import argparse
import json
import math
import tarfile
import urllib.request
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from susumu_face_engagement_detector.backends.recognition import make_backend


CHOKEPOINT_RECORD = "https://zenodo.org/records/815657"
CHOKEPOINT_API = "https://zenodo.org/api/records/815657/files"
CHOKEPOINT_LICENSE = "CC-BY-NC-4.0"
DEFAULT_OUT_DIR = Path("outputs/runs/chokepoint")
DEFAULT_ARCHIVE = "P1E.tar.xz"
DEFAULT_SEQUENCE = "P1E_S1_C1"


@dataclass
class GalleryEntry:
    person_id: str
    embedding: np.ndarray


@dataclass(frozen=True)
class IdentificationResult:
    pred_id: str
    distance: Optional[float]
    nearest_id: Optional[str]
    nearest_distance: Optional[float]
    second_id: Optional[str]
    second_distance: Optional[float]
    reason: str


def _download(url: str, destination: Path) -> Path:
    if destination.exists() and destination.stat().st_size > 0:
        print(f"using existing file: {destination}")
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    tmp = destination.with_suffix(destination.suffix + ".tmp")
    print(f"downloading: {url}")
    with urllib.request.urlopen(url, timeout=120) as response, tmp.open("wb") as fh:
        total = int(response.headers.get("Content-Length") or 0)
        copied = 0
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            fh.write(chunk)
            copied += len(chunk)
            if total:
                print(
                    f"  {copied / 1024 / 1024:.1f}/{total / 1024 / 1024:.1f} MiB",
                    end="\r",
                )
    if total:
        print()
    tmp.replace(destination)
    return destination


def _safe_extract(tar: tarfile.TarFile, destination: Path) -> None:
    root = destination.resolve()
    for member in tar.getmembers():
        target = (destination / member.name).resolve()
        if target != root and root not in target.parents:
            raise RuntimeError(f"unsafe archive path: {member.name}")
    tar.extractall(destination)


def _ensure_dataset(out_dir: Path, archive_name: str) -> Path:
    archive_path = out_dir / archive_name
    archive_url = f"{CHOKEPOINT_API}/{archive_name}/content"
    _download(archive_url, archive_path)
    extract_dir = out_dir / archive_name.removesuffix(".tar.xz")
    marker = extract_dir / ".extracted"
    if not marker.exists():
        extract_dir.mkdir(parents=True, exist_ok=True)
        print(f"extracting: {archive_path}")
        with tarfile.open(archive_path, "r:xz") as tar:
            _safe_extract(tar, extract_dir)
        marker.write_text("ok\n")
    return extract_dir


def _collect_tracks(root: Path, sequence: str, min_frames: int) -> Dict[str, List[Path]]:
    seq_dir = root / sequence
    if not seq_dir.exists():
        raise FileNotFoundError(seq_dir)
    tracks: Dict[str, List[Path]] = {}
    for person_dir in sorted(seq_dir.iterdir()):
        if not person_dir.is_dir() or not person_dir.name.isdigit():
            continue
        frames = sorted(person_dir.glob("*.pgm"))
        if len(frames) >= min_frames:
            tracks[person_dir.name] = frames
    if not tracks:
        raise RuntimeError(f"no tracks with at least {min_frames} frames in {seq_dir}")
    return tracks


def _embed_image(backend, path: Path) -> Optional[np.ndarray]:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    return backend.embed(rgb, (0, w, h, 0))


def _build_gallery(backend, tracks: Dict[str, List[Path]], gallery_frames: int) -> List[GalleryEntry]:
    gallery: List[GalleryEntry] = []
    for person_id, frames in tracks.items():
        embeddings = []
        for path in frames[:gallery_frames]:
            emb = _embed_image(backend, path)
            if emb is not None:
                embeddings.append(emb)
        if embeddings:
            gallery.append(GalleryEntry(person_id, np.mean(embeddings, axis=0)))
    if not gallery:
        raise RuntimeError("failed to build any gallery embeddings")
    return gallery


def _identify(
    gallery: List[GalleryEntry],
    embedding: Optional[np.ndarray],
    tolerance: float,
    previous_id: Optional[str] = None,
    identity_margin: float = 0.15,
) -> IdentificationResult:
    if embedding is None:
        return IdentificationResult("unknown", None, None, None, None, None, "no_embedding")

    distances = []
    for item in gallery:
        dist = float(np.linalg.norm(item.embedding - embedding))
        distances.append((dist, item.person_id))
    distances.sort()

    if not distances:
        return IdentificationResult("unknown", None, None, None, None, None, "empty_gallery")

    best_dist, best_id = distances[0]
    second_dist, second_id = distances[1] if len(distances) > 1 else (None, None)
    if best_dist > tolerance:
        return IdentificationResult(
            "unknown",
            best_dist,
            best_id,
            best_dist,
            second_id,
            second_dist,
            "over_tolerance",
        )

    if previous_id and previous_id != best_id:
        margin = max(0.0, identity_margin)
        for dist, person_id in distances:
            if person_id != previous_id:
                continue
            if dist <= tolerance and dist <= best_dist + margin:
                return IdentificationResult(
                    previous_id,
                    dist,
                    best_id,
                    best_dist,
                    second_id,
                    second_dist,
                    "previous_margin",
                )
            break

    return IdentificationResult(
        best_id,
        best_dist,
        best_id,
        best_dist,
        second_id,
        second_dist,
        "nearest",
    )


def _draw_text(
    frame: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.55,
    color: Tuple[int, int, int] = (245, 248, 250),
) -> None:
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (8, 10, 12), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _open_writer(path: Path, fps: float, size: Tuple[int, int]) -> cv2.VideoWriter:
    path.parent.mkdir(parents=True, exist_ok=True)
    for codec in ("mp4v", "avc1", "MJPG"):
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), fps, size)
        if writer.isOpened():
            return writer
        writer.release()
    raise RuntimeError(f"failed to open writer: {path}")


def run(args: argparse.Namespace) -> Path:
    out_dir = Path(args.out_dir)
    dataset_root = _ensure_dataset(out_dir, args.archive)
    tracks = _collect_tracks(dataset_root, args.sequence, args.gallery_frames + 2)
    selected_ids = sorted(tracks)[: args.max_identities]
    tracks = {pid: tracks[pid] for pid in selected_ids}

    backend = make_backend(args.recognition_backend)
    gallery = _build_gallery(backend, tracks, args.gallery_frames)
    gallery_ids = {entry.person_id for entry in gallery}
    tracks = {pid: frames for pid, frames in tracks.items() if pid in gallery_ids}
    ids = sorted(tracks)

    if not ids:
        raise RuntimeError("no identities remained after gallery construction")

    fps = float(args.fps)
    total_frames = max(1, int(round(args.seconds * fps)))
    cols = min(args.columns, len(ids))
    rows = int(math.ceil(args.visible_identities / cols))
    face_size = int(args.face_size)
    tile_w = face_size + 34
    tile_h = face_size + 78
    width = cols * tile_w + 30
    height = rows * tile_h + 112
    output = Path(args.output)
    writer = _open_writer(output, fps, (width, height))

    frame_records = []
    counters = defaultdict(int)
    previous_predictions: Dict[str, str] = {}
    visible = min(args.visible_identities, len(ids))
    switch_every = max(1, int(round(args.switch_seconds * fps)))

    try:
        for frame_idx in range(total_frames):
            canvas = np.full((height, width, 3), 34, dtype=np.uint8)
            _draw_text(
                canvas,
                f"ChokePoint GT face ID | {args.sequence}",
                16,
                28,
                0.62,
                (230, 235, 240),
            )
            _draw_text(
                canvas,
                f"{CHOKEPOINT_LICENSE} | {args.recognition_backend} | tol={args.tolerance:.2f} margin={args.identity_margin:.2f}",
                16,
                54,
                0.48,
                (210, 216, 224),
            )
            _draw_text(
                canvas,
                "green=correct red=wrong gray=unknown",
                16,
                78,
                0.48,
                (210, 216, 224),
            )

            group_start = ((frame_idx // switch_every) * visible) % len(ids)
            visible_ids = [ids[(group_start + i) % len(ids)] for i in range(visible)]
            entries = []
            for slot, person_id in enumerate(visible_ids):
                row = slot // cols
                col = slot % cols
                x = 16 + col * tile_w
                y = 100 + row * tile_h
                frames = tracks[person_id]
                probe_frames = frames[args.gallery_frames :] or frames
                path = probe_frames[frame_idx % len(probe_frames)]
                bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                embedding = _embed_image(backend, path)
                result = _identify(
                    gallery,
                    embedding,
                    args.tolerance,
                    previous_id=previous_predictions.get(person_id),
                    identity_margin=args.identity_margin,
                )
                pred_id = result.pred_id
                distance = result.distance
                if pred_id != "unknown":
                    previous_predictions[person_id] = pred_id
                correct = pred_id == person_id
                if correct:
                    color = (80, 220, 120)
                    counters["correct"] += 1
                elif pred_id == "unknown":
                    color = (170, 176, 184)
                    counters["unknown"] += 1
                else:
                    color = (80, 80, 235)
                    counters["wrong"] += 1

                face = cv2.resize(bgr, (face_size, face_size), interpolation=cv2.INTER_CUBIC)
                cv2.rectangle(canvas, (x - 2, y - 2), (x + face_size + 2, y + face_size + 2), color, 3)
                canvas[y : y + face_size, x : x + face_size] = face
                _draw_text(canvas, f"GT {person_id}", x, y + face_size + 24, 0.52, (245, 248, 250))
                dist_text = "n/a" if distance is None else f"{distance:.2f}"
                _draw_text(canvas, f"pred {pred_id} d={dist_text}", x, y + face_size + 48, 0.45, color)

                counters["total"] += 1
                entries.append(
                    {
                        "slot": slot,
                        "bbox": [x, y, x + face_size, y + face_size],
                        "gt_id": person_id,
                        "pred_id": pred_id,
                        "distance": distance,
                        "nearest_id": result.nearest_id,
                        "nearest_distance": result.nearest_distance,
                        "second_id": result.second_id,
                        "second_distance": result.second_distance,
                        "selection_reason": result.reason,
                        "correct": correct,
                        "source_frame": path.name,
                    }
                )

            writer.write(canvas)
            frame_records.append({"frame": frame_idx, "faces": entries})
            if frame_idx == 0 or (frame_idx + 1) % max(1, int(fps)) == 0:
                print(f"processed {frame_idx + 1}/{total_frames} frames")
    finally:
        writer.release()

    total = int(counters["total"])
    metrics = {
        "total": total,
        "correct": int(counters["correct"]),
        "wrong": int(counters["wrong"]),
        "unknown": int(counters["unknown"]),
        "accuracy": (float(counters["correct"]) / total) if total else 0.0,
    }
    sidecar = output.with_suffix(output.suffix + ".json")
    sidecar.write_text(
        json.dumps(
            {
                "dataset": "ChokePoint",
                "source": CHOKEPOINT_RECORD,
                "license": CHOKEPOINT_LICENSE,
                "archive": args.archive,
                "sequence": args.sequence,
                "output": str(output),
                "fps": fps,
                "frames_written": total_frames,
                "recognition_backend": args.recognition_backend,
                "tolerance": args.tolerance,
                "identity_margin": args.identity_margin,
                "gallery_frames": args.gallery_frames,
                "identities": ids,
                "metrics": metrics,
                "frames": frame_records,
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )
    print(f"wrote GT video: {output}")
    print(f"wrote GT metadata: {sidecar}")
    print(f"metrics: {metrics}")
    return output


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create a ChokePoint ground-truth face-ID video.")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--archive", default=DEFAULT_ARCHIVE)
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--output", default=str(DEFAULT_OUT_DIR / "chokepoint_gt_face_id_demo.mp4"))
    parser.add_argument("--recognition-backend", default="dlib_128d", choices=["dlib_128d"])
    parser.add_argument("--tolerance", type=float, default=0.75)
    parser.add_argument(
        "--identity-margin",
        type=float,
        default=0.15,
        help="Prefer the previously displayed ID when it is within best_distance + this margin.",
    )
    parser.add_argument("--gallery-frames", type=int, default=5)
    parser.add_argument("--max-identities", type=int, default=12)
    parser.add_argument("--visible-identities", type=int, default=6)
    parser.add_argument("--columns", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--switch-seconds", type=float, default=5.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--face-size", type=int, default=180)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        run(args)
    except Exception as exc:
        print(f"error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
