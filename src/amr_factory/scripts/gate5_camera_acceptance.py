#!/usr/bin/env python3

"""Validate exact-time RGB-D camera delivery and expected AprilTag IDs."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_key(message) -> tuple[int, int]:
    return (message.header.stamp.sec, message.header.stamp.nanosec)


class CameraAcceptance(Node):
    def __init__(self, expected_ids: set[int], capture_path: Path) -> None:
        super().__init__("gate5_camera_acceptance")
        self.expected_ids = expected_ids
        self.capture_path = capture_path
        self.images = {}
        self.infos = {}
        self.paired_stamps = []
        self.depth_received = False
        self.detected_ids = set()
        self.create_subscription(
            Image,
            "/amr/sensors/product_camera/image_rect",
            self._image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/amr/sensors/product_camera/camera_info",
            self._info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/amr/sensors/product_camera/depth",
            self._depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            "/amr/perception/product_tags",
            self._detections,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _valid_camera_message(message) -> bool:
        return (
            message.width == 640
            and message.height == 480
            and message.header.frame_id == "product_camera_optical_frame"
        )

    def _image(self, message: Image) -> None:
        if not self._valid_camera_message(message):
            return
        key = stamp_key(message)
        self.images[key] = message
        if not self.capture_path.exists():
            self._write_capture(message)
        self._record_pair(key)
        self._trim(self.images)

    def _info(self, message: CameraInfo) -> None:
        if not self._valid_camera_message(message):
            return
        key = stamp_key(message)
        self.infos[key] = message
        self._record_pair(key)
        self._trim(self.infos)

    def _depth(self, message: Image) -> None:
        self.depth_received = self._valid_camera_message(message)

    def _detections(self, message: AprilTagDetectionArray) -> None:
        for detection in message.detections:
            if detection.family == "tag36h11" and detection.hamming == 0:
                self.detected_ids.add(detection.id)

    def _record_pair(self, key: tuple[int, int]) -> None:
        if key not in self.images or key not in self.infos:
            return
        if key not in self.paired_stamps:
            self.paired_stamps.append(key)
        del self.images[key]
        del self.infos[key]

    @staticmethod
    def _trim(messages: dict) -> None:
        while len(messages) > 20:
            del messages[min(messages)]

    def _write_capture(self, message: Image) -> None:
        if message.encoding not in ("rgb8", "bgr8"):
            return
        rows = []
        for row in range(message.height):
            start = row * message.step
            pixels = bytearray(message.data[start:start + message.width * 3])
            if message.encoding == "bgr8":
                for index in range(0, len(pixels), 3):
                    pixels[index], pixels[index + 2] = pixels[index + 2], pixels[index]
            rows.append(pixels)
        with self.capture_path.open("wb") as stream:
            stream.write(f"P6\n{message.width} {message.height}\n255\n".encode("ascii"))
            for row in rows:
                stream.write(row)

    def accepted(self) -> bool:
        return (
            len(self.paired_stamps) >= 5
            and self.depth_received
            and self.expected_ids <= self.detected_ids
        )

    def verify_timing(self) -> None:
        stamps = sorted(
            second + nanosecond / 1_000_000_000.0
            for second, nanosecond in self.paired_stamps[-6:]
        )
        intervals = [right - left for left, right in zip(stamps, stamps[1:])]
        quantized = [round(interval / 0.1) for interval in intervals]
        if (
            len(intervals) < 4
            or any(multiplier < 1 for multiplier in quantized)
            or any(abs(interval - multiplier * 0.1) > 1e-6
                    for interval, multiplier in zip(intervals, quantized))
        ):
            raise RuntimeError(
                "camera timestamps are not on the configured 0.1 s cadence: "
                f"{intervals}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected", type=int, nargs="*", default=[])
    parser.add_argument(
        "--capture", type=Path, default=Path("/tmp/phase14_gate5_camera.ppm")
    )
    arguments = parser.parse_args()
    rclpy.init()
    node = CameraAcceptance(set(arguments.expected), arguments.capture)
    deadline = time.monotonic() + 15.0
    try:
        while time.monotonic() < deadline and not node.accepted():
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.paired_stamps) < 5:
            raise RuntimeError("fewer than five exact-time image/CameraInfo pairs")
        node.verify_timing()
        if not node.depth_received:
            raise RuntimeError("no valid depth image")
        missing = node.expected_ids - node.detected_ids
        if missing:
            raise RuntimeError(
                f"missing expected zero-hamming tag IDs {sorted(missing)}; "
                f"observed {sorted(node.detected_ids)}; capture {arguments.capture}"
            )
        print(
            "PASS Gate 5 camera: 10 Hz-grid simulation timestamps, valid depth, "
            f"zero-hamming tags {sorted(node.detected_ids)}"
        )
        return 0
    except Exception as error:
        print(f"FAIL Gate 5 camera acceptance: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
