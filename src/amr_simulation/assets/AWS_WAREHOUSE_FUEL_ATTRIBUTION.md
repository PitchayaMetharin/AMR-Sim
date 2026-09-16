# AWS warehouse Fuel attribution

The packaged `aws_warehouse.sdf` is a deterministic SDF transformation of the
official OpenRobotics Fuel world source:

<https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/industrial-warehouse/4/files/industrial-warehouse.sdf>

- Source owner: OpenRobotics (Fuel).
- Source revision: `industrial-warehouse` revision `4`.
- Source size: `7522` bytes.
- Source SHA-256: `a80de9f6d76e6b589a8b080ccdbce87bd1b88cce9b88cf9de6c81af10602c161`.
- Verified: `2026-09-15`.

The SDF references these exact Fuel resources. Every resource is owned by
OpenRobotics on Fuel; the revision is part of each URL.

| Resource | Exact Fuel URL |
| --- | --- |
| `aws_robomaker_warehouse_Bucket_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_Bucket_01/3> |
| `aws_robomaker_warehouse_ShelfF_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfF_01/4> |
| `aws_robomaker_warehouse_WallB_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_WallB_01/4> |
| `aws_robomaker_warehouse_ShelfE_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfE_01/4> |
| `aws_robomaker_warehouse_ShelfD_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfD_01/4> |
| `aws_robomaker_warehouse_GroundB_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_GroundB_01/4> |
| `aws_robomaker_warehouse_Lamp_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_Lamp_01/4> |
| `aws_robomaker_warehouse_ClutteringA_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringA_01/4> |
| `aws_robomaker_warehouse_ClutteringC_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringC_01/4> |
| `aws_robomaker_warehouse_ClutteringD_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringD_01/4> |
| `aws_robomaker_warehouse_TrashCanC_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_TrashCanC_01/4> |
| `aws_robomaker_warehouse_PalletJackB_01` | <https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_PalletJackB_01/4> |

The packaged file contains SDF references only; Fuel model bundles are not
vendored. On first use, resolve the canonical Fuel host with DNS/TLS and use
the network unless the exact canonical host, owner, model, and revision are
already cached. Once every exact resource revision is cached, that cache may
be reused offline. A cache under the legacy
`fuel.ignitionrobotics.org` host is not claimed equivalent until runtime
verification. The remote prerequisite (Fuel) remains remote and is not vendored.

The resource set is Amazon-originated through the archived AWS RoboMaker
upstream, but per-resource Fuel metadata is authoritative for this packaged
world. The existing factory attribution and MIT-0 statement applies only to
the four vendored factory models; it does not cover this remote 12-model bundle.
No remote model license is asserted or copied here.
