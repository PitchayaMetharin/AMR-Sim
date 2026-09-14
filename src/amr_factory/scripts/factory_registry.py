"""Validated factory station/product registry helpers.

The YAML files are the single source of truth for station routing, product
selection, autonomous admission, and dispatch-slot assignment.  This module is
kept dependency-light so the CLI and launch-time contract tests can use the
same validation rules without starting ROS or Gazebo.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import yaml


class RegistryError(ValueError):
    """A malformed or internally inconsistent factory registry."""


Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class Station:
    station_id: str
    role: str
    approach: Pose
    dock: Optional[Pose]
    egress: Optional[Pose]


@dataclass(frozen=True)
class Product:
    product_id: str
    model: str
    tag_id: int
    mass_kg: float
    pickup_station: str
    autonomous_enabled: bool
    dispatch_slot: str


@dataclass(frozen=True)
class FactoryRegistry:
    stations: Mapping[str, Station]
    products: Mapping[str, Product]
    dispatch_slots: Mapping[str, Tuple[float, float, float]]

    def station(self, station_id: str) -> Station:
        try:
            return self.stations[station_id]
        except KeyError as error:
            raise RegistryError(f"unknown station: {station_id}") from error

    def product_for_station(self, station_id: str, autonomous_only: bool = False) -> Product:
        matches = [
            product for product in self.products.values()
            if product.pickup_station == station_id and
            (not autonomous_only or product.autonomous_enabled)
        ]
        if len(matches) != 1:
            qualifier = " autonomous-enabled" if autonomous_only else ""
            raise RegistryError(
                f"station {station_id} does not resolve to exactly one{qualifier} product")
        return matches[0]


def _default_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("amr_factory")) / "config"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config"


def _finite_pose(value: Any, label: str, allow_none: bool = False) -> Optional[Pose]:
    if value is None and allow_none:
        return None
    if not isinstance(value, Mapping):
        raise RegistryError(f"{label} must be a pose mapping")
    try:
        pose = tuple(float(value[key]) for key in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError) as error:
        raise RegistryError(f"{label} is invalid") from error
    if not all(math.isfinite(item) for item in pose):
        raise RegistryError(f"{label} contains a non-finite value")
    return pose  # type: ignore[return-value]


def load_registry(config_dir: Optional[Path] = None) -> FactoryRegistry:
    """Load and validate both registry files."""
    root = Path(config_dir) if config_dir is not None else _default_config_dir()
    try:
        with (root / "stations.yaml").open(encoding="utf-8") as stream:
            station_yaml = yaml.safe_load(stream) or {}
        with (root / "products.yaml").open(encoding="utf-8") as stream:
            product_yaml = yaml.safe_load(stream) or {}
    except OSError as error:
        raise RegistryError("factory station or product registry is unavailable") from error

    raw_stations = station_yaml.get("stations")
    raw_products = product_yaml.get("products")
    raw_slots = product_yaml.get("dispatch_slots")
    if not isinstance(raw_stations, Mapping) or not isinstance(raw_products, Mapping):
        raise RegistryError("stations and products must be mappings")
    if not isinstance(raw_slots, list) or not raw_slots:
        raise RegistryError("dispatch_slots must be a non-empty list")

    stations: Dict[str, Station] = {}
    role_counts: Dict[str, int] = {}
    station_tag_ids = set()
    for station_id, raw in raw_stations.items():
        if not isinstance(station_id, str) or not station_id or not isinstance(raw, Mapping):
            raise RegistryError("station entries must have string IDs and mappings")
        role = raw.get("role")
        if role not in {"home", "pickup", "dispatch"}:
            raise RegistryError(f"station {station_id} has an invalid role")
        approach = _finite_pose(raw.get("approach"), f"{station_id}.approach")
        dock = _finite_pose(raw.get("dock"), f"{station_id}.dock", allow_none=True)
        egress = _finite_pose(raw.get("egress"), f"{station_id}.egress", allow_none=True)
        if role == "pickup" and (dock is None or egress is None):
            raise RegistryError(f"pickup station {station_id} needs dock and egress poses")
        if role == "home" and (dock is not None or egress is not None):
            raise RegistryError(f"home station {station_id} must not have dock or egress poses")
        if role == "dispatch" and dock is None:
            raise RegistryError(f"dispatch station {station_id} needs a dock pose")
        if role == "dispatch" and egress is not None:
            raise RegistryError(f"dispatch station {station_id} must not have an egress pose")
        raw_tag_id = raw.get("tag_id")
        if raw_tag_id is not None:
            if isinstance(raw_tag_id, bool) or not isinstance(raw_tag_id, int):
                raise RegistryError(f"station {station_id} tag_id must be an integer")
            if raw_tag_id <= 0 or raw_tag_id in station_tag_ids:
                raise RegistryError(f"duplicate or invalid station tag_id: {station_id}")
            station_tag_ids.add(raw_tag_id)
        stations[station_id] = Station(station_id, str(role), approach, dock, egress)
        role_counts[str(role)] = role_counts.get(str(role), 0) + 1
    if role_counts.get("home") != 1 or role_counts.get("dispatch") != 1:
        raise RegistryError("registry must contain exactly one home and one dispatch station")

    slots: Dict[str, Tuple[float, float, float]] = {}
    for raw in raw_slots:
        if not isinstance(raw, Mapping) or not isinstance(raw.get("id"), str) or not raw["id"]:
            raise RegistryError("dispatch slot is missing an ID")
        slot_id = raw["id"]
        if slot_id in slots:
            raise RegistryError(f"duplicate dispatch slot: {slot_id}")
        try:
            position = tuple(float(raw[key]) for key in ("x", "y", "z"))
        except (KeyError, TypeError, ValueError) as error:
            raise RegistryError(f"dispatch slot {slot_id} is invalid") from error
        if not all(math.isfinite(item) for item in position):
            raise RegistryError(f"dispatch slot {slot_id} contains a non-finite value")
        slots[slot_id] = position  # type: ignore[assignment]

    products: Dict[str, Product] = {}
    tag_ids = set()
    pickup_ids = set()
    for model, raw in raw_products.items():
        if not isinstance(model, str) or not isinstance(raw, Mapping):
            raise RegistryError("product entries must have string model names and mappings")
        try:
            raw_tag_id = raw["tag_id"]
            if isinstance(raw_tag_id, bool) or not isinstance(raw_tag_id, int):
                raise RegistryError(f"product {model} tag_id must be an integer")
            tag_id = raw_tag_id
            raw_mass = raw["mass"]
            if isinstance(raw_mass, bool):
                raise RegistryError(f"product {model} mass must be numeric")
            mass_kg = float(raw_mass)
            pickup_station = str(raw["pickup_station"])
            autonomous_value = raw["autonomous_enabled"]
            if not isinstance(autonomous_value, bool):
                raise RegistryError(
                    f"product {model} autonomous_enabled must be a boolean")
            autonomous_enabled = autonomous_value
            dispatch_slot = str(raw["dispatch_slot"])
        except (KeyError, TypeError, ValueError) as error:
            raise RegistryError(f"product {model} is missing required registry fields") from error
        if tag_id in tag_ids:
            raise RegistryError(f"duplicate product tag ID: {tag_id}")
        if not math.isfinite(mass_kg) or mass_kg <= 0.0:
            raise RegistryError(f"product {model} mass is invalid")
        station = stations.get(pickup_station)
        if station is None or station.role != "pickup":
            raise RegistryError(f"product {model} references a non-pickup station")
        if dispatch_slot not in slots:
            raise RegistryError(f"product {model} references missing dispatch slot {dispatch_slot}")
        if pickup_station in pickup_ids:
            raise RegistryError(f"multiple products reference pickup station {pickup_station}")
        if tag_id == 103 and autonomous_enabled:
            raise RegistryError("product 103 must remain disabled for autonomous work")
        tag_ids.add(tag_id)
        pickup_ids.add(pickup_station)
        products[str(tag_id)] = Product(
            str(tag_id), model, tag_id, mass_kg, pickup_station,
            autonomous_enabled, dispatch_slot)
    if not products:
        raise RegistryError("registry has no products")
    return FactoryRegistry(stations, products, slots)


def autonomous_product_for_station(station_id: str) -> Product:
    """Resolve a station only when its product is enabled for autonomous work."""
    return load_registry().product_for_station(station_id, autonomous_only=True)
