#!/usr/bin/env python3
"""Derive deterministic ROS 2 meshes from the legacy AMR CAD export.

The SolidWorks export is intentionally kept outside the ROS 2 package.  This
script gates the verified v2 chassis, LiDAR and caster provenance before the
legacy conversion step, preserving all thirteen output files byte-for-byte:

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
from collections import Counter
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


# Approved v2 source/component identities; visual equivalence does not authorize
# importing its repeated assembly, baked arm, camera joint, or inertias.
V2_RAW_MESHES = Path("amr_v2") / "meshes"
V2_IDENTITIES = {
    "base_link.STL": (
        "2d168ef5fd088472cfbf7c89d2675cb2504042dca1eca21f56fed64d30946a4b",
        "b0f27db25987905634d2bff27f52c68fd1fe90f1ce1d0977ec57de63eb44d015", 111, (
            (10, 3838, (-408000, -253000, -20000), (408000, 253000, 330000)),
            (11, 4536, (-405000, -250000, -20000), (405000, 250000, 165838)),
            (12, 96, (-400000, -80000, -20000), (-220000, -40000, 20000)),
            (13, 96, (-400000, 40000, -20000), (-220000, 80000, 20000)),
            (14, 96, (-220000, -245000, -20000), (-180000, 245000, 20000)),
            (15, 320, (-180000, -120000, -20000), (220000, 120000, 20000)),
            (16, 96, (220000, -80000, -20000), (400000, -40000, 20000)),
            (17, 96, (220000, 40000, -20000), (400000, 80000, 20000)),
            (18, 1024, (-387000, -232000, 23000), (269688, 232000, 198000)),
            (20, 284, (308000, 150317, 37000), (388022, 233000, 40000)),
            (21, 128, (-27368, -167501, 41917), (25895, -142501, 62000)),
            (22, 128, (-27368, 142500, 41917), (25895, 167500, 62000)),
            (27, 2384, (116000, -182501, 71024), (144000, -173501, 98976)),
            (28, 2384, (116000, -159501, 71024), (144000, -150501, 98976)),
            (29, 2384, (116000, -136501, 71024), (144000, -127501, 98976)),
            (30, 2384, (116000, 127500, 71024), (144000, 136500, 98976)),
            (31, 2384, (116000, 150500, 71024), (144000, 159500, 98976)),
            (32, 2384, (116000, 173500, 71024), (144000, 182500, 98976)),
            (33, 144, (126000, -182501, 81061), (134000, -127501, 88939)),
            (34, 144, (126000, 127500, 81061), (134000, 182500, 88939)),
            (35, 816, (87010, -177501, 137862), (143046, -132501, 234587)),
            (36, 816, (87010, 132500, 137862), (143046, 177500, 234587)),
            (37, 2548, (122000, -180001, 142000), (138000, -175001, 158000)),
            (38, 2548, (122000, -135001, 142000), (138000, -130001, 158000)),
            (39, 2548, (122000, 130000, 142000), (138000, 135000, 158000)),
            (40, 2548, (122000, 175000, 142000), (138000, 180000, 158000)),
            (41, 144, (126000, -180001, 146061), (134000, -130001, 153939)),
            (42, 144, (126000, 130000, 146061), (134000, 180000, 153939)),
            (43, 6940, (66134, -173610, 164256), (132527, -136393, 267167)),
            (44, 6940, (66134, 136391, 164256), (132527, 173609, 267167)),
            (45, 8024, (55512, -168501, 187959), (120972, -141501, 293773)),
            (46, 8024, (55512, 141500, 187959), (120972, 168500, 293773)),
            (47, 1362, (-28500, -20950, 198000), (28500, 20950, 221600)),
            (48, 2460, (118518, 253000, 230518), (177494, 253500, 289482)),
            (49, 1170, (129494, 227058, 237657), (166506, 241110, 288871)),
            (50, 1514, (128003, 241100, 240000), (167997, 278950, 280000)),
            (51, 3678, (131219, 207400, 241210), (147007, 235025, 282391)),
            (52, 3644, (129486, 207400, 242070), (145274, 235025, 283251)),
            (53, 1368, (130883, 219100, 245271), (140554, 221400, 254705)),
            (54, 2054, (132719, 208900, 246988), (138718, 219400, 252988)),
            (55, 3490, (52891, -180902, 248627), (102379, -129101, 275927)),
            (56, 3490, (52891, 129099, 248627), (102379, 180901, 275927)),
            (57, 12, (141168, 212700, 255503), (141924, 213900, 258051)),
            (58, 12, (141168, 225300, 255503), (141924, 226500, 258051)),
            (59, 12, (132759, 212700, 257083), (133515, 213900, 259632)),
            (60, 12, (132759, 225300, 257083), (133515, 226500, 259632)),
            (61, 608, (133995, 226073, 259300), (142497, 233900, 265161)),
            (62, 12, (142978, 212700, 264829), (143734, 213900, 267378)),
            (63, 12, (142978, 225300, 264829), (143734, 226500, 267378)),
            (64, 12, (134568, 212700, 266409), (135324, 213900, 268958)),
            (65, 12, (134568, 225300, 266409), (135324, 226500, 268958)),
            (66, 1368, (135644, 219100, 269813), (145315, 221400, 279247)),
            (67, 2054, (137480, 208900, 271531), (143479, 219400, 277530)),
            (68, 452, (23, -172501, 272647), (77579, -137501, 315757)),
        )),
    "lidar_front_link.STL": (
        "72e0538007caa1e96a95d89a546f0bc063d8f6332fe0d42a93b0bd536f9e2523",
        "737dc0e77d3624833792781a6a0ece65711039ab123ce93ca1e2cbfcbc8b3176", 113, (
            (72, 24109, (-74048, -50892, -158719), (50892, 74044, 3065)),
            (75, 152, (-48185, 3166, -148219), (-47086, 4265, -147719)),
            (78, 70, (32149, -37244, -66579), (32616, -36835, -62686)),
            (80, 1320, (-49009, -48932, -54686), (48262, 48365, -54470)),
            (84, 152, (-30881, 26677, -54132), (-26685, 30874, -52102)),
            (85, 152, (-12627, -41413, -54132), (-8431, -37216, -52102)),
            (86, 152, (37214, 8441, -54132), (41410, 12637, -52102)),
        )),
    "lidar_back_link.STL": (
        "806b377a345acd610c4d8d2cff22bfabb46543caa53f140c2c5b771857806162",
        "90689416da03a53c39e551fa3ed06ecf463574ec95ba3989c30e3d5e3e660054", 111, (
            (70, 24109, (-74048, -50892, -158719), (50892, 74044, 3065)),
            (73, 152, (-48185, 3166, -148219), (-47086, 4265, -147719)),
            (76, 70, (32149, -37244, -66579), (32616, -36835, -62686)),
            (78, 1320, (-49009, -48932, -54686), (48262, 48365, -54470)),
            (82, 152, (-30881, 26677, -54132), (-26685, 30874, -52102)),
            (83, 152, (-12627, -41413, -54132), (-8431, -37216, -52102)),
            (84, 152, (37214, 8441, -54132), (41410, 12637, -52102)),
        )),
    "left_caster_front_link.STL": (
        "e72fd31b8311073b98cae9ca7ead0612a60d6e35fde535468e30dd7e68273741",
        "9dcd8083014c0abb9ea07e5eb4f32364134a82ad8e2bb607cca6ccc06455671d", 111, (
            (5, 3360, (-62441, -20955, -94233), (12441, 21110, -19367)),
            (9, 2004, (-44076, -35000, -68850), (35000, 35000, 2200)),
            (26, 3670, (-45118, -45118, 0), (45118, 45118, 11700)),
        )),
    "right_caster_front_link.STL": (
        "8a66a92f90af43af7e2094f0befa3aecbbafd762fad10078eab9b582e4786887",
        "02461a39b7cd3b74cf66a71f5fb11835ab435244452c3fdeef7f90a69cccad84", 113, (
            (4, 3360, (-62441, -21110, -102433), (12441, 20955, -27567)),
            (8, 2004, (-44076, -35000, -77050), (35000, 35000, -6000)),
            (25, 3670, (-45118, -45118, -8200), (45118, 45118, 3500)),
        )),
    "left_caster_back_link.STL": (
        "7813d350e7af5d3e9142e4e30429e7f7ebebef5b4eae9f9b61440deb20789fe7",
        "f687ef11870a423b67671ba9a11727a25394c90550f9ffc79051c20c2b65dc17", 111, (
            (3, 3360, (-12441, -20955, -94233), (62441, 21110, -19367)),
            (7, 2004, (-35000, -35000, -68850), (44076, 35000, 2200)),
            (24, 3670, (-45118, -45118, 0), (45118, 45118, 11700)),
        )),
    "right_caster_back_link.STL": (
        "afec0b3a83e86aec415ea036b11e3175bf7e58da870a280cce35f2ec5ad3424a",
        "a8595cf64b6da5fe3b75c1e8efde57a4c654123523e4e205217ae9452ceaa0b0", 111, (
            (2, 3360, (-12441, -21110, -94233), (62441, 20955, -19367)),
            (6, 2004, (-35000, -35000, -68850), (44076, 35000, 2200)),
            (23, 3670, (-45118, -45118, 0), (45118, 45118, 11700)),
        )),
}


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


def oriented_records(records: tuple[bytes, ...]) -> Counter:
    """Compare cyclic winding, normals, attributes and triangle multiplicity."""
    signatures = []
    for record in records:
        values = struct.unpack("<12fH", record)
        vertices = tuple(vertex_key(vertex) for vertex in record_vertices(record))
        winding = min(vertices, vertices[1:] + vertices[:1], vertices[2:] + vertices[:2])
        signatures.append((vertex_key(values[:3]), winding, values[12]))
    return Counter(signatures)


def validate_v2_records(source_name: str, records: tuple[bytes, ...],
                        component_count: int, mapping: tuple,
                        reference_records: tuple[bytes, ...]) -> None:
    if len(records) != 266_696:
        fail(f"{source_name} v2 triangle count changed: expected 266696, got {len(records)}")
    components = connected_components(records)
    if len(components) != component_count:
        fail(f"{source_name} v2 component count changed: expected {component_count}, got {len(components)}")
    selected = []
    for index, count, lower, upper in mapping:
        component = components[index]
        if (component.triangle_count, component.lower, component.upper) != (count, lower, upper):
            fail(f"{source_name} v2 mapped component {index} changed")
        selected.extend(records[i] for i in component.triangle_indices)
    if oriented_records(tuple(selected)) != oriented_records(reference_records):
        fail(f"{source_name} v2 oriented geometry, normals, attributes or multiplicity changed")


def validate_v2_sources(root: Path) -> None:
    """Gate v2 provenance against legacy source derivation, never output meshes."""
    for name, (v2_sha, legacy_sha, component_count, mapping) in V2_IDENTITIES.items():
        source = root / V2_RAW_MESHES / name
        legacy = root / RAW_MESHES / name
        for path, expected_sha in ((source, v2_sha), (legacy, legacy_sha)):
            require_file(path)
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha:
                fail(f"{path} SHA256 changed: expected {expected_sha}")
        _, records = read_binary_stl(source)
        if name == "base_link.STL":
            _, reference_bytes = derive_base(root)
            reference_count = struct.unpack_from("<I", reference_bytes, 80)[0]
            reference = tuple(reference_bytes[84 + 50*i:84 + 50*(i+1)]
                              for i in range(reference_count))
        else:
            _, reference = read_binary_stl(legacy)
        validate_v2_records(name, records, component_count, mapping, reference)


def expected_outputs(root: Path) -> dict[Path, bytes]:
    validate_v2_sources(root)
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
