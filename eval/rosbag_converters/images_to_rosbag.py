"""Convert an iterable of images into a rosbag2 (mcap) of sensor_msgs/Image.

Used to feed datasets (WIDER FACE, LFW, custom recordings, ...) into the
detection / recognition pipeline without depending on a physical camera.

Each image is stamped with a synthetic time (configurable fps) and a stable
image_id is also written into a parallel std_msgs/String topic so that the
evaluation runner can correlate predictions with ground truth even when bag
playback reorders messages slightly.

Usage (CLI):
    python3 -m eval.rosbag_converters.images_to_rosbag \
        --image-list path/to/list.txt \
        --output /tmp/wider_val.bag \
        --topic /camera/color/image_raw \
        --fps 5.0
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Tuple

import cv2
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _open_writer(output: Path, storage_id: str) -> rosbag2_py.SequentialWriter:
    if output.exists():
        # rosbag2 refuses to overwrite — caller should remove explicitly.
        raise FileExistsError(f"Bag path already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    storage = rosbag2_py.StorageOptions(uri=str(output), storage_id=storage_id)
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    writer.open(storage, conv)
    return writer


def _register_topics(writer: rosbag2_py.SequentialWriter, image_topic: str, id_topic: str) -> None:
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=image_topic,
            type="sensor_msgs/msg/Image",
            serialization_format="cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=id_topic,
            type="std_msgs/msg/String",
            serialization_format="cdr",
        )
    )


def write_images_to_bag(
    image_iter: Iterable[Tuple[str, Path]],
    output: Path,
    image_topic: str = "/camera/color/image_raw",
    id_topic: str = "/eval/image_id",
    fps: float = 5.0,
    frame_id: str = "camera_color_optical_frame",
    storage_id: str = "mcap",
    start_time_ns: int = 1_000_000_000,
) -> int:
    """Write images into a rosbag2.

    image_iter yields (image_id, image_path) pairs in playback order.
    Returns the number of images written.
    """
    if fps <= 0:
        raise ValueError("fps must be > 0")
    period_ns = int(1e9 / fps)

    writer = _open_writer(output, storage_id)
    _register_topics(writer, image_topic, id_topic)

    bridge = CvBridge()
    t_ns = start_time_ns
    count = 0
    skipped = 0
    for image_id, image_path in image_iter:
        img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if img is None:
            skipped += 1
            continue
        msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = t_ns // 1_000_000_000
        msg.header.stamp.nanosec = t_ns % 1_000_000_000

        id_msg = String(data=image_id)

        writer.write(image_topic, serialize_message(msg), t_ns)
        writer.write(id_topic, serialize_message(id_msg), t_ns)

        t_ns += period_ns
        count += 1

    if skipped:
        print(f"[images_to_rosbag] WARNING: skipped {skipped} unreadable images")
    return count


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument(
        "--image-list",
        type=Path,
        help="Text file with one '<image_id>\\t<image_path>' per line.",
    )
    g.add_argument(
        "--wider-val-root",
        type=Path,
        help="Use the WIDER FACE val set rooted here.",
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--topic", default="/camera/color/image_raw")
    p.add_argument("--id-topic", default="/eval/image_id")
    p.add_argument("--fps", type=float, default=5.0)
    p.add_argument("--storage-id", default="mcap", choices=["mcap", "sqlite3"])
    p.add_argument("--limit", type=int, default=0, help="Stop after N images (0=all)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    def _iter() -> Iterable[Tuple[str, Path]]:
        if args.image_list:
            with args.image_list.open() as f:
                for line in f:
                    line = line.rstrip("\n")
                    if not line or line.startswith("#"):
                        continue
                    image_id, image_path = line.split("\t", 1)
                    yield image_id, Path(image_path)
        else:
            from eval.datasets.wider_face import load_val
            entries = load_val(args.wider_val_root)
            for e in entries:
                yield e.image_id, e.image_path

    def _limited(it: Iterable[Tuple[str, Path]]) -> Iterable[Tuple[str, Path]]:
        if args.limit <= 0:
            yield from it
            return
        for i, item in enumerate(it):
            if i >= args.limit:
                return
            yield item

    n = write_images_to_bag(
        _limited(_iter()),
        output=args.output,
        image_topic=args.topic,
        id_topic=args.id_topic,
        fps=args.fps,
        storage_id=args.storage_id,
    )
    print(f"[images_to_rosbag] wrote {n} images to {args.output}")


if __name__ == "__main__":
    main()
