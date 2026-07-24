# Phase 0 Safety Scope

## Project boundary

The current project is a simulation-only academic AMR project executed on a
laptop. It does not include physical construction, procurement, installation,
commissioning, or industrial deployment.

No functional-safety certification, Performance Level, SIL, conformity, or
industrial-suitability claim will be made from the simulation. Simulated PLC
authority, emergency stopping, protective stopping, bumper inputs, contactors,
and motion permissives demonstrate architecture and state behavior only.

The SICK MRS1000 units and their simulated data are operational navigation
perception. They are not safety-rated personnel-protection devices.

## Jurisdiction and reference standards

The intended future jurisdiction is Thailand. The conceptual architecture uses
international references so it is not limited to a Thailand-specific
deployment:

- [ISO 3691-4:2023](https://www.iso.org/standard/83545.html) — Industrial
  trucks — Safety requirements and verification — Part 4: Driverless
  industrial trucks and their systems.
- [ISO 12100:2010](https://www.iso.org/standard/51528.html) — Safety of
  machinery — General principles for design — Risk assessment and risk
  reduction.
- [ISO 13849-1:2023](https://www.iso.org/standard/73481.html) — Safety of
  machinery — Safety-related parts of control systems — Part 1: General
  principles for design.
- [IEC 60204-1:2016+A1:2021](https://webstore.iec.ch/en/publication/13923) —
  Safety of machinery — Electrical equipment of machines — Part 1: General
  requirements.

These references were verified against the official ISO and IEC catalogs on
2026-07-24. Access to the full standards and a documented risk assessment
would be required for a physical design.

## Provisional conceptual target

PL d, Category 3 may be used as a provisional architecture target for critical
future physical safety functions such as:

- emergency stopping;
- protective stopping;
- prevention of unintended motion;
- drive-power/permission removal and feedback monitoring.

This provisional target is not a derived required Performance Level
(`PLr`). ISO 13849-1 does not prescribe the PLr for a particular application.
Each safety function's PLr must be derived from the future physical machine's
documented risk assessment and validated using the final hardware, diagnostic
coverage, common-cause measures, failure data, wiring, software, and test
evidence.

## Validation authority

For this academic simulation:

- project-team and university-supervisor review is the validation authority;
- review verifies consistency, simulated state behavior, documented
  assumptions, and test evidence;
- it is not formal machinery-safety validation.

Before physical construction or industrial deployment:

- a competent machinery-safety engineer must perform and document the risk
  assessment;
- the safety-function specification, PLr determination, design, calculations,
  and validation must be completed;
- formal review by an accredited assessor or certification body is required
  where applicable.
