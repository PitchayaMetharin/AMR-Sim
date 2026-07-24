# Phase 0 BOM Review

## Source

- File: `Industrial_AMR_BOM_with_Thailand_Suppliers_Prices.xlsx`
- Workbook check date recorded in the file: 2026-07-17
- Reviewed in the workspace: 2026-07-24
- Sheets: `Full BOM`, `Power Budget`, `Communication Matrix`,
  `Design Notes`, and `Procurement Summary`
- Full BOM grain: one procurement line per component/side/function
- Full BOM size after architecture update: 38 component rows and 24 columns

The workbook was updated on 2026-07-24 to remove the obsolete sensing plan. It
is still not safe for procurement until the remaining row-integrity issues and
provisional selections below are resolved.

The current project is simulation-only. The workbook is retained as a
conceptual future physical implementation reference; no procurement is planned.
Supplier and pricing remediation is explicitly deferred.

## Architecture Conflicts

| Severity | BOM content | Governing project requirement | Disposition |
|---|---|---|---|
| Resolved | BOM initially contained legacy navigation LiDARs | 2 × SICK MRS1104C-111011 / 1081208 | Current BOM contains two exact MRS1104C-111011 lines with official electrical data. |
| Resolved | BOM initially contained 2 × SICK outdoorScan3 safety scanners and PROFIsafe paths | outdoorScan3 removed; current architecture has no safety LiDAR | Obsolete BOM, communication, and design-note entries removed. |
| Resolved | BOM initially contained 2 × TENTE swivel casters | Four passive casters: front-left, front-right, rear-left, rear-right | Current BOM quantity is four TENTE LEVINA 5370PJP100P62 units; dimensions and load suitability remain unverified. |
| Provisional | 24 V power budget depended on obsolete sensor loads | Power budget must reflect the approved dual-MRS1000 architecture | Current verified component values total 174 W; 25% margin requires 217.5 W from the selected 240 W supply, leaving 22.5 W. Recheck simultaneous states, wiring loss, and final variants. |
| Resolved | Communication matrix initially contained legacy sensor paths | Matrix must contain two MRS1104C-111011 data paths | Current matrix identifies both exact sensors and their verified Ethernet/CoLa interface; ROS driver and bench validation remain open. |
| Medium | Design notes state that manufacturer CAD is needed for the simulation assembly | Phase 6 requires a primitive, parameterized URDF/Xacro; user owns mechanical CAD | Treat CAD links as optional packaging references, not as a prerequisite or manufacturing source. |

## Data-Quality Findings

### Critical: supplier/procurement fields are shifted

From at least Full BOM row 32 onward, several `Preferred Supplier`, `Shop
Link`, price, and availability fields describe a neighboring component rather
than the component on the same row.

Examples:

- `XB5AA31` points to the `XB5AS8445` shop listing and repeats its price.
- `XB5AG33` points to the `XB5AA31` shop listing and price.
- `SE-P40` points to an `XB5AG33` listing.
- `LR6-302WJNW-RYG` carries the Schmersal safety-edge supplier/link.
- `BKV-24` carries an LR6 signal-tower listing.
- `SCALANCE W774-1` carries a PATLITE supplier/link.
- `E3Z-D62 2M` carries a Siemens SCALANCE listing.
- `ROBOFIX` carries an Omron E3Z listing and price.
- `SK 3237.100` carries a SCHUNK ROBOFIX listing.
- The caster row carries a Rittal thermostat listing.
- The enclosure row carries the TENTE caster listing.

Impact: procurement source, price, availability, and the THB subtotal are not
reliable for the affected rows.

### High: procurement summary remains unreliable

- The Procurement Summary includes prices attached to the wrong components.
- The 24 V budget has been rebuilt from the current component rows, but final
  accessory loads, wiring loss, simultaneity, and delivered variants still
  require Phase 2 verification.
- The traction-bus budget remains a rated-power sum rather than a validated
  drive/regeneration/thermal/endurance analysis.

### High: exact-model completeness is mixed

Several rows provide exact ordering codes, but others contain a family,
provisional value, or unresolved suffix:

- charger;
- main fuse and rating;
- safety-contactor coil/auxiliary-contact variant;
- safety-edge length;
- Wi-Fi regional/radio variant;
- charging-contact current class;
- cabinet fan sizing;
- enclosure sizing;
- caster dimensions and final load suitability;
- battery/BMS manufacturer, ordering code, and interfaces.

Impact: these entries are architecture candidates, not purchase-ready lines.

### Medium: compatibility claims are not evidence

The Communication Matrix uses values such as `Yes` and `Yes conceptually`
without recording an official document revision, tested firmware combination,
connector/pinout evidence, or validation record.

Impact: the claims may guide investigation, but they cannot close an interface
or safety requirement.

## Useful Exact Identifiers Recovered

These identifiers can seed official manufacturer verification after the user
confirms that the corresponding BOM line remains selected:

| Function | BOM identifier | Current status |
|---|---|---|
| Compute | NVIDIA Jetson Orin Nano Developer Kit 8GB | Matches frozen family; verify production suitability and exact kit revision. |
| Motor driver | ZLTECH ZLAC8030D | Matches frozen selection; official revision/firmware verification required. |
| Drive motors | 2 × ZLTECH ZLLG10ASM800 V2.0 | Official product page confirms 10-inch nominal wheel; remaining mechanical/electrical/encoder data require verification. |
| IMU | Xsens MTi-8-5A-DK | Matches MTi-8 family; confirm development-kit versus production module intent. |
| Fail-safe PLC | Siemens 6ES7214-1AF40-0XB0 | Future physical S7-1200F candidate only. Phase 1 uses an S7-1500F virtual PLC; exact simulated model remains TBD. |
| Fail-safe input | Siemens 6ES7226-6BA32-0XB0 | Candidate; Phase 1/2 responsibility and I/O count verification required. |
| Fail-safe output | Siemens 6ES7226-6DA32-0XB0 | Candidate; Phase 1/2 responsibility and I/O count verification required. |
| Ethernet switch | Siemens 6GK5216-0BA00-2AC2 / SCALANCE XC216 | Matches frozen family; port-speed/topology suitability remains to be validated. |
| CAN interface | PEAK PCAN-USB Pro FD IPEH-004061 | Candidate; ZLAC CANopen compatibility and deployment support must be verified. |
| HMI | Siemens 6AV2123-2GB03-0AX0 / KTP700 Basic PN | Candidate; application need and I/O/network architecture remain unapproved. |
| LiDARs | 2 × SICK MRS1104C-111011 / 1081208 | Exact variant and official 10–30 VDC / 37 W maximum data verified; firmware/driver/bench validation open. |
| Casters | 4 × TENTE LEVINA 5370PJP100P62 | Quantity reconciled; geometry/load suitability unverified. |

## Official MRS1000 Verification

SICK's official current variant list includes, among others:

- `MRS1104C-111011`, order number `1081208`, identified by SICK as the outdoor
  variant;
- `MRS1104C-011010`, order number `1075367`, identified as the indoor variant;
- `MRS1104C-111011S02`, order number `1106288`, identified as the 32-field
  variant;
- `MRS1104C-911011S05`, order number `1131433`, identified as the heavy-duty
  variant with stainless-steel connector.

Sources:

- [SICK MRS1000 variants / numbers](https://support.sick.com/sick-knowledgebase/article/?code=KA-07193)
- [SICK MRS1000 product family overview](https://www.sick.com/media/familyoverview/2/52/152/familyOverview_MRS1000_g387152_en.pdf)

The user selected `MRS1104C-111011`, order number `1081208`, for both units.
SICK's exact current datasheet identifies indoor/outdoor use, 10–30 VDC input,
13 W typical consumption, 37 W maximum consumption, a one-second 30 W startup
maximum, four scan layers, 275° horizontal aperture, and 7.5° vertical
aperture.

- [Exact SICK MRS1104C-111011 / 1081208 datasheet](https://www.sick.com/media/pdf/4/44/044/dataSheet_MRS1104C-111011_1081208_en.pdf)

## Official Drive-Wheel Verification

ZLTECH's exact product page identifies `ZLLG10ASM800 V2.0` as a 10-inch hub
wheel. This establishes a 0.254 m nominal diameter and 0.127 m provisional
nominal radius. It does not establish the effective rolling radius under load
or validate a 300 kg AMR payload.

- [Official ZLTECH ZLLG10ASM800 V2.0 product page](https://www.zlingkj.com/en/robot-hub-servo-motor-series/565267)

## Required Remediation Before Hardware-Dependent Implementation

The following applies only if a future physical project is authorized:

1. Correct the shifted supplier/shop/price/status cells against the intended
   component rows.
2. Recalculate procurement totals after bad mappings are
   removed.
3. Revalidate the power budget from final worst-case simultaneous operating
   loads and wiring/converter losses.
4. Complete the communication matrix with the selected ROS 2 driver, firmware,
   IP addressing, and time-synchronization design.
5. Replace remaining generic compatibility assertions with official-source citations and
   later bench-validation evidence.
6. Complete unresolved part suffixes and ordering codes before procurement.

The sensing-architecture cleanup changed the workbook on 2026-07-24 at the
user's direction. A pre-update recovery copy is held at
`/tmp/Industrial_AMR_BOM_pre_MRS1000_update_2026-07-24.xlsx` for the current
workspace session. A second recovery point before applying the confirmed
MRS1000/caster/power values is held at
`/tmp/Industrial_AMR_BOM_pre_confirmed_requirements_2026-07-24.xlsx`. A third
recovery point before applying the simulation-only labels is held at
`/tmp/Industrial_AMR_BOM_pre_simulation_only_scope_2026-07-24.xlsx`.
