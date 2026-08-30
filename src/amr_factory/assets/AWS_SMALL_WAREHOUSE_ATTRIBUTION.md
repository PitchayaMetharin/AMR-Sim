# AWS Small Warehouse asset attribution

The local `models/` directory contains a deliberately limited port of assets
from `aws-robotics/aws-robomaker-small-warehouse-world`, branch `ros2`, commit
`ee0af733315e78432408c3cd98d378ecee5f767c`:

- `aws_robomaker_warehouse_ShelfD_01`;
- `aws_robomaker_warehouse_PalletJackB_01`;
- `aws_robomaker_warehouse_ClutteringA_01`; and
- `aws_robomaker_warehouse_Bucket_01`.

The source is licensed under MIT-0; its license is retained in
`AWS_SMALL_WAREHOUSE_LICENSE`. The model SDF mesh URIs were converted from
source-tree-relative `file://models/...` references to local `model://...`
references for Gazebo Harmonic. Mesh and texture contents are otherwise
unchanged. Runtime does not contact Gazebo Fuel or another network service.
