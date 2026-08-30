#!/usr/bin/env python3
"""Derive deterministic ROS 2 meshes from the legacy AMR CAD export.

The SolidWorks export is intentionally kept outside the ROS 2 package.  This
script is the single, fail-closed conversion step for its visual meshes:

* the base export is hash- and topology-gated before its baked arm, mounting
  plate, and pedestal geometry are excluded;
* drive and LiDAR assets are copied byte-for-byte; and
* each caster is split into its two body components and recentered wheel.

Only Python's standard library is used so the derivation can run before a ROS
workspace is built.  ``--check`` compares every expected output with the
deterministic bytes that would be generated without changing any file.
"""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
from dataclasses import dataclass
from pathlib import Path


QUANTIZATION = 1_000_000
BASE_SHA256 = "b0f27db25987905634d2bff27f52c68fd1fe90f1ce1d0977ec57de63eb44d015"
BASE_TRIANGLES = 128_670
BASE_COMPONENTS = 80
KEPT_BASE_COMPONENTS = 54
KEPT_BASE_TRIANGLES = 96_178
REMOVED_BASE_TRIANGLES = 32_492

RAW_MESHES = Path("amr_urdf_cad") / "meshes"
DERIVED_MESHES = Path("src") / "amr_description" / "meshes"

COPY_ASSETS = (
    "left_wheel_link.STL",
    "right_wheel_link.STL",
    "lidar_front_link.STL",
    "lidar_back_link.STL",
)
CASTER_ASSETS = (
    "left_caster_front_link.STL",
    "left_caster_back_link.STL",
    "right_caster_front_link.STL",
    "right_caster_back_link.STL",
)


@dataclass(frozen=True)
class Component:
    """A connected STL component and its quantized bounds."""

    triangle_indices: tuple[int, ...]
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]

    @property
    def triangle_count(self) -> int:
        return len(self.triangle_indices)


def fail(message: str) -> "NoReturn":
    raise RuntimeError(message)


def quantize(value: float) -> int:
    if not isinstance(value, float):
        value = float(value)
    if not (value == value and abs(value) != float("inf")):
        fail("STL contains a non-finite vertex")
    return int(round(value * QUANTIZATION))


def vertex_key(values: tuple[float, float, float]) -> tuple[int, int, int]:
    return tuple(quantize(value) for value in values)


def read_binary_stl(path: Path) -> tuple[bytes, tuple[bytes, ...]]:
    data = path.read_bytes()
    if len(data) < 84:
        fail(f"{path} is not a complete binary STL")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if len(data) != expected_size:
        fail(f"{path} has an invalid binary STL size")
    records = tuple(data[offset : offset + 50]
                    for offset in range(84, expected_size, 50))
    return data[:80], records


def record_vertices(record: bytes) -> tuple[tuple[float, float, float], ...]:
    values = struct.unpack("<12fH", record)
    return (tuple(values[3:6]), tuple(values[6:9]), tuple(values[9:12]))


def connected_components(records: tuple[bytes, ...]) -> tuple[Component, ...]:
    """Return components sorted by quantized lower Z, then X/Y and source order."""

    parent: list[int] = []
    rank: list[int] = []
    vertex_ids: dict[tuple[int, int, int], int] = {}

    def new_vertex(key: tuple[int, int, int]) -> int:
        vertex_id = vertex_ids.get(key)
        if vertex_id is not None:
            return vertex_id
        vertex_id = len(parent)
        vertex_ids[key] = vertex_id
        parent.append(vertex_id)
        rank.append(0)
        return vertex_id

    def find(vertex_id: int) -> int:
        while parent[vertex_id] != vertex_id:
            parent[vertex_id] = parent[parent[vertex_id]]
            vertex_id = parent[vertex_id]
        return vertex_id

    def union(first: int, second: int) -> None:
        first = find(first)
        second = find(second)
        if first == second:
            return
        if rank[first] < rank[second]:
            first, second = second, first
        parent[second] = first
        if rank[first] == rank[second]:
            rank[first] += 1

    triangle_vertex_keys: list[tuple[tuple[int, int, int], ...]] = []
    for record in records:
        keys = tuple(vertex_key(vertex) for vertex in record_vertices(record))
        triangle_vertex_keys.append(keys)
        ids = tuple(new_vertex(key) for key in keys)
        union(ids[0], ids[1])
        union(ids[1], ids[2])
        union(ids[2], ids[0])

    triangle_groups: dict[int, list[int]] = {}
    for triangle_index, keys in enumerate(triangle_vertex_keys):
        root = find(vertex_ids[keys[0]])
        triangle_groups.setdefault(root, []).append(triangle_index)

    components: list[Component] = []
    for triangle_indices in triangle_groups.values():
        lower = [None, None, None]
        upper = [None, None, None]
        for triangle_index in triangle_indices:
            for vertex in record_vertices(records[triangle_index]):
                key = vertex_key(vertex)
                for axis in range(3):
                    lower[axis] = key[axis] if lower[axis] is None else min(lower[axis], key[axis])
                    upper[axis] = key[axis] if upper[axis] is None else max(upper[axis], key[axis])
        components.append(Component(
            tuple(triangle_indices),
            tuple(value for value in lower if value is not None),
            tuple(value for value in upper if value is not None),
        ))

    components.sort(key=lambda component: (
        component.lower[2],
        component.lower[0],
        component.lower[1],
        component.triangle_indices[0],
    ))
    return tuple(components)


def stl_bytes(header: bytes, records: tuple[bytes, ...]) -> bytes:
    output_header = (header[:80] + b"\0" * 80)[:80]
    return output_header + struct.pack("<I", len(records)) + b"".join(records)


def require_file(path: Path) -> None:
    if not path.is_file():
        fail(f"required CAD asset is missing: {path}")


def derive_base(root: Path) -> tuple[Path, bytes]:
    source = root / RAW_MESHES / "base_link.STL"
    require_file(source)
    source_bytes = source.read_bytes()
    digest = hashlib.sha256(source_bytes).hexdigest()
    if digest != BASE_SHA256:
        fail(f"base_link.STL SHA256 changed: expected {BASE_SHA256}, got {digest}")
    header, records = read_binary_stl(source)
    if len(records) != BASE_TRIANGLES:
        fail(f"base_link.STL triangle count changed: expected {BASE_TRIANGLES}, got {len(records)}")
    components = connected_components(records)
    if len(components) != BASE_COMPONENTS:
        fail(f"base_link.STL component count changed: expected {BASE_COMPONENTS}, got {len(components)}")
    if sum(component.triangle_count for component in components[:KEPT_BASE_COMPONENTS]) != KEPT_BASE_TRIANGLES:
        fail("base_link.STL retained component triangle count changed")
    if sum(component.triangle_count for component in components[KEPT_BASE_COMPONENTS:]) != REMOVED_BASE_TRIANGLES:
        fail("base_link.STL removed component triangle count changed")
    # Components 55 and 56 are the excluded mounting plate and tall centered
    # pedestal.  Check both explicitly so a changed export cannot silently
    # put either component back into the derived visual.
    plate = components[KEPT_BASE_COMPONENTS]
    if (plate.triangle_count != 506
            or plate.lower != (-119_327, -151_469, 325_757)
            or plate.upper != (152_000, 151_469, 353_757)):
        fail("base_link.STL excluded mounting plate component changed")
    pedestal = components[KEPT_BASE_COMPONENTS + 1]
    if pedestal.triangle_count != 1_180 or pedestal.lower[2] != 353_757 or pedestal.upper[2] != 542_639:
        fail("base_link.STL centered pedestal component changed")

    kept_indices = sorted(index for component in components[:KEPT_BASE_COMPONENTS]
                          for index in component.triangle_indices)
    derived_records = tuple(records[index] for index in kept_indices)
    if len(derived_records) != KEPT_BASE_TRIANGLES:
        fail("base_link.STL derivation produced an unexpected triangle count")
    return root / DERIVED_MESHES / "base_link.STL", stl_bytes(header, derived_records)


def derive_caster(root: Path, source_name: str) -> tuple[tuple[Path, bytes], tuple[Path, bytes]]:
    source = root / RAW_MESHES / source_name
    require_file(source)
    header, records = read_binary_stl(source)
    if len(records) != 9_034:
        fail(f"{source_name} triangle count changed: expected 9034, got {len(records)}")
    # The export writes the two body surfaces first and the wheel last.  Keep
    # that source order for the caster split; unlike the base, its wheel is
    # intentionally the lowest-Z component.
    components = tuple(sorted(
        connected_components(records),
        key=lambda component: component.triangle_indices[0],
    ))
    counts = tuple(component.triangle_count for component in components)
    if counts != (3_670, 2_004, 3_360):
        fail(f"{source_name} component counts changed: expected (3670, 2004, 3360), got {counts}")

    body_indices = sorted(index for component in components[:2]
                          for index in component.triangle_indices)
    wheel_indices = sorted(components[2].triangle_indices)
    body_records = tuple(records[index] for index in body_indices)
    wheel_component = components[2]
    translation = tuple(
        (wheel_component.lower[axis] + wheel_component.upper[axis]) / (2 * QUANTIZATION)
        for axis in range(3)
    )
    wheel_records: list[bytes] = []
    for index in wheel_indices:
        record = records[index]
        values = list(struct.unpack("<12fH", record))
        for vertex_start in (3, 6, 9):
            for axis in range(3):
                values[vertex_start + axis] -= translation[axis]
        wheel_records.append(struct.pack("<12fH", *values))

    stem = source_name.removesuffix(".STL")
    return (
        root / DERIVED_MESHES / f"{stem}_body.STL",
        stl_bytes(header, body_records),
    ), (
        root / DERIVED_MESHES / f"{stem}_wheel.STL",
        stl_bytes(header, tuple(wheel_records)),
    )


def expected_outputs(root: Path) -> dict[Path, bytes]:
    outputs: dict[Path, bytes] = {}
    base_path, base_data = derive_base(root)
    outputs[base_path] = base_data
    for name in COPY_ASSETS:
        source = root / RAW_MESHES / name
        require_file(source)
        outputs[root / DERIVED_MESHES / name] = source.read_bytes()
    for name in CASTER_ASSETS:
        for path, data in derive_caster(root, name):
            outputs[path] = data
    return outputs


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="verify outputs without modifying files")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[3],
                        help="workspace root (default: repository containing this script)")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    outputs = expected_outputs(root)
    if args.check:
        mismatches = []
        for path, data in sorted(outputs.items()):
            if not path.is_file() or path.read_bytes() != data:
                mismatches.append(path)
        if mismatches:
            for path in mismatches:
                print(f"out of date: {path}", file=sys.stderr)
            return 1
        print(f"CAD mesh outputs are deterministic and up to date ({len(outputs)} files)")
        return 0

    output_dir = root / DERIVED_MESHES
    output_dir.mkdir(parents=True, exist_ok=True)
    for path, data in sorted(outputs.items()):
        path.write_bytes(data)
    print(f"derived {len(outputs)} deterministic CAD mesh files under {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except RuntimeError as error:
        print(f"derive_cad_meshes.py: error: {error}", file=sys.stderr)
        raise SystemExit(1)
