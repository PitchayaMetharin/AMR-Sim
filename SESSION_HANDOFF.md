# AMR Session Handoff

## USER PAUSED — GUI24 functional PASS; placement/departure stability open — 2026-10-01

User-requested read-only recheck completed: result.json confirms17/17 gates,
all owned exits0, survivors=[]; live process audit found no simulator, runner,
MoveIt launch, recorder, observer or stability decoder. Analyzer interrupted,
workers completed. Final proof homeXY0.009540654m/yaw0.001497449rad; finalslot
A0.0000060364m/B0.0000191488m, all10 finalproof gates true. Current allocated
logs274169856/evidence424865792bytes; logical222732588/413810514, bothcapsPASS.
Protected handoff SHA unchanged; provisional stability report38lines before
adding these finalproof figures. Remaining user observations and pause intact.

User is leaving and explicitly requests handoff only. STOP: no new diagnosis,
source edits, bag decoding, runtime, delegation or automatic continuation until
explicit user resume. HIGH analyzer interrupted; other workers completed.
Authoritative run24 exec77612 finishedexit0 during handoff verification; no
owned runtime survivors. Preserve current candidate and all evidence as-is;
no rollback or acceptance decision was made after stop.

GUI24 old-model command after sourcing Humble/workspace:
`PYTHONDONTWRITEBYTECODE=1 python3 phase14_evidence/factory_runtime_tools/run_full_factory.py --run-id factory_full_validation_20261001_24 --domain 232 --gui`
All17 recorded gates exit0, success:true; both product cycles/home, final proof,
finalized recorder and unchanged strict analyzers pass. All owned teardown
exit0; survivors=[]. Exact result and final proof in run24 evidence directory.
Functional PASS is separate from controller optimization acceptance.

User says controller/stability is "a lot better", but still observes:
1. During placing, AMR enters green placing zone, turns strongly left/right,
   advances while apparently tilting, then readjusts position.
2. After placing A and reversing/turning toward pickup B, repeated forward/
   backward motion and a stationary delay remain.
3. Approaching A place zone, deceleration appears too early; tiny creeping
   movements continue for a while. These are priority observations to correlate
   with planned maneuvers/controller response, not dismissed by passing gates.

Only current controller packet changed precision approachfloor0.01 to0.02m/s
and matching existing contract assertion (11tests/build/installedparity PASS).
Old robot geometry/axes/arm/camera/dynamics and every safety gate unchanged.
CAD replacement remains deferred. No command syntax change; full/quick amr_cmd
references already contain --gui. Their run-status text still names GUI23;
updating it can wait for explicit resume alongside final quantitative review.

<=60line provisional stability report: stability_20261001/run24/stability_report.md.
Fresh per-phase errors/reversals/overshoot/settling/full-quaternion tipping and
microleg speed comparison remain UNEXTRACTED at user stop; retain full finalized
GUI24bag plus GUI23 baseline/cache. Startup RTF aggregate0.996732/median0.999993.
HIGH found strict experimental <=5ms terminal acquisition alignment UNPROVEN:
FollowPath status/result lack terminalstamp, GoalInfo is acceptance time,
feedback TF stamp is pose acquisition; stamped mission terminal comes after
async result delivery. Receipt proximity cannot bound DDS delay. Original
terminal comparison is INCONCLUSIVE absent causal proof; do not loosen bands
or claim optimization accepted. Independent source review remains pending.

Storage from runner final snapshot: logical logs222732588/evidence413808210;
allocated logs274169856/evidence424861696 bytes, both below500000000 including
retained artifacts. Small report/handoff additions fit. No active decoder.
Protected AMR_CODEX_HANDOFF.md untouched; unrelated dirty work preserved.

Safe resume after explicit instruction: recheck state/storage; sole MEDIUM
bounded finalized-bag analysis and <=60line numerical report, HIGH diagnosis
of these placement/departure/creeping observations and timestamp limitations,
then independent review/acceptance decision before any further controller edit.

---

## Successful source task — precision floor candidate ready — 2026-10-01

gpt-6.1-sol/medium navigation_trace changed only PlacementFollowPath approach
floor0.01 to0.02m/s in src/amr_mpc_controller/config/controller.yaml and its
matching existing assertion in test/test_mpc_controller_contract.py. Focused
pytest11passedexit0; parsed YAML baseline comparison exactlyone scalar; complete
scoped diff inspected/diffcheckexit0; sourced scoped colconbuildexit0; installed
YAML byte parity SHA76389b60577e8165c02aa7774db96c23d4fd78963c73541cadb3e42f435e8c36.
All other parameters, active old model and gates unchanged. Source/build success
is not runtime or independent acceptance. Root next GUI24/domain232, with first
mandatory failure stop/cleanup then HIGH diagnosis and packet comparisons.

---

## Current authority — old-model controller stabilization resumed — 2026-10-01

User explicitly resumed controller/stabilization work using the OLD active model.
CAD replacement, arm substitution and camera reparenting are deferred. Preserve
active geometry, axes, dynamics, camera and all existing safety/ownership gates.
Root completed TWO full reads of this handoff (2047 lines before this checkpoint).
Latest user `continue` authorizes proceeding; older paused/CAD/AWS directives
below are history and do not replace this current scope.

GUI23 remains the functional baseline: runner exit0, all17 mandatory recorded
gates pass, clean owned teardown. Its outside-yaw heading branch was unexercised;
earlier recovery collision and the user's reported prolonged hunting are not
claimed resolved. Baseline stability report is43lines under stability_20261001/run23.

Completed HIGH diagnosis task: precision_approach_packet.md (58lines) supports
one-variable PlacementFollowPath approach-floor experiment .01 to .02m/s.
Actual installed RPP probe matches measured5.75–5.85s short-leg tails and predicts
~3.5s. MEDIUM navigation_trace owns exactly controller.yaml and its existing
contract assertion, then focused checks/build; root owns fresh GUI24/domain232.
Performance acceptance requires >=20% reduction from17.403s for three comparable
A microlegs, no added precision reversals/hunting/contact/tipping, unchanged
mandatory gates and the packet's stricter experimental docking-error ceilings.
Noncomparable data is INCONCLUSIVE; no optimization claim before runtime/review.

Explicit task delegation remains gpt-6.1-sol/medium workers and high analyzer,
one production writer, progress<=120s (root<=60s), immediate successful-task
handoff updates, every simulation stability report<=60lines. Retained logs and
evidence each total<=500,000,000bytes logical AND allocated, temporary data
included. Last allocated logs269,361,152/evidence346,578,944;150MBreserve passed.
No runtime currently active. Protected AMR_CODEX_HANDOFF.md remains untouched.

---

## Latest retention checkpoint — preserve GUI baseline for optimization — 2026-10-01

Userstandingdisposable-workspaceauthorization used to retire26supersededrun22
compressedbagparts, redundantrun22numericalcache and completedGUI23shadercache.
Allreports/metrics/source/CAD retained; run22metadata explicitlymarkedpurged.
FULLGUI23baselinebag+numericalcache remainsavailable for candidatecomparison.
ExactSHA/size/removalreceipt stability_20261001/superseded_run22_removal.json;
commandexit0,77,961,581bytesremoved. Allocatedlogs269,361,152/evidence346,578,944,
logical218,210,190/336,162,590bytes; separate500MBcaps and150MBreservepass.

Userclarifies cameraarmmount was anassumption andisopeningCADtocheck. Intended
camera parent/relativepose remainsunconfirmed; do notassumearmreparenting.
Currentactivearm retains6workingaxes/joints; rawv2hasbakedarmbasevisual only.

---

## Latest successful task — guarded amr_v2 geometry provenance — 2026-10-01

Mediumworker changedexact3approvedfiles: description derive_cad_meshes.py,
test_description.py and docs/PHASE_14_CAD_URDF_REMEDIATION_REPORT.md. Mandatory
v2SHA/topology/mappedbound-count/orientedvertices/normals/attributes/multiplicity
admission runs before legacyreferencebytepreservingderivation. Missingoutputs
regenerate fromsource; all13active meshSHAremainidentical. Real--checkexit0,
newCADtests9pass, description35pass/1existingstaleprobe127. Highdiagnosed
binaryMoveIt2.5.9versusinstalled2.5.10; approvedscopedforceconfigurebuildexit0
relinked2.5.10, failedprobe-onlypytest1passexit0. Base/compositeXacro/check_urdf
andscopeddiffexit0. Independentreviewpending. NoactiveURDF/axis/camera/arm/
collision/dynamicschange; thisis partialprovenance, NOTfullmodelreplacement.
Cameraarmparent/relativepose stillawaituser; existingbasecamera datausability
notestablished by factoryrun23 (rawimage/depthstreamsnotrecorded).

---

## Latest checkpoint — recorded Gazebo GUI run23 PASS — 2026-10-01

Exact command: `PYTHONDONTWRITEBYTECODE=1 python3
phase14_evidence/factory_runtime_tools/run_full_factory.py --run-id
factory_full_validation_20261001_23 --domain 232 --gui` after Humble/workspace
setup. Authoritative session2056 completedexit0; result.jsonsuccessTrue and
all17recordedmandatorygatesexit0, including finalizedrecorder and bothunchangedstrict
analyzers. Product101,Product102,home and finalphysical/native/factory proof
passed. Everyownedteardownexit0 and survivors=[]; GUIclient73356 hadrunon:0.
FinalhomeXY0.0080801087m/yaw0.0011962103rad; slotA3.7001913e-6m,
slotB5.5317458e-5m. StartupaggregateRTF0.996327/median1.000019.

Coverage limitation: pickupB precise ingress endedyaw0.0709rad, alreadywithin
.15rad. The newoutside-yaw headingcontroller branch was NOT exercised by this
runtime; behavioralregressions coverit, but pathologicalGUIrecovery remains
unverified. Do not claim all reported oscillation or modelreplacement fixed.
Mediumworker owns finalizedrun23 quantitative stability extraction (<=60line
report); highanalyzer independently reviews authoredNAVdiff afterfreshGUIrun.
CADworker completes safeexactcomponentpacket; originalCAD/activegeometry held
and cameraarmparent/transform stillawaituser. NoactiveGazebo remains.

Completed command-reference task: full workspace reference (and its amr_cmd
symlink) plus /home/pete/sh&text/amr_cmd/SIMULATION_COMMANDS_FACTORY_QUICK.md
now show identical owned `--gui` workflow, omitflagforheadless, actualrun23
17gatePASS pluscleanteardown and explicitpathologicalheadingbranch/runtimecoverage limitation.
All Bashblocks syntaxchecked and full/quickworkflowidentical. Originalmanual
workflow retained. This outside-workspace document path is alreadyexplicitly
authorized by user's earlier full/quickcommand update request.

Completed run23 lossless retention task: soledecoderreleased, thenall26bag
parts streamingzstd19reencoded withdecodedSHA/bytecountverified before atomic
replace, exit0. Exactreceipt retention_20261001/bag_reencoding_receipt.json.
Fullrun22/run23baselinesretained. Corrected count is17recordedgates plusowned
teardown (earliercurrent-task18wording was an accountingerror, not extra gate).

Completed run23 stability analysis task (medium):43line
`stability_20261001/run23/gui_stability_report.md`, metrics/comparisonJSON and
bounded numerical cache retained. All1,066,897finalizedmessagesverified;
decoder/analyzer/assertionsexit0, tempremoved. JobsA180.20s/B172.20s versus
baseline180.40s/171.80s; reversaldistributionunchanged, no within-leglinear
reversals/terminalhunting/tipping. First3A microsegments~.070meach take5.85,
5.75,5.80s, each~2.95sat.010m/sexistingapproachspeedfloor; thisis performance
evidence for analyzer consideration, not an optimizationPASS. Both dockyaws
.08119/.07089alreadywithin.15; exceptionalnewbranch remainsunexercised.

Completed NAV independent source review (high):57line
`stability_20261001/navigation_independent_review.md`; own41pytest/AST/diffcheck
exit0 and genuineHEADbaselinestalerejectionprobe failsasrequired. No material
sourcefinding; exactpacket/cancellation/safetygatespreserved. GUIfunctional
PASS doesnotverify outside-yawbranch or resolve earlierrecoverycollision.

---

## Latest successful task — navigation capability/freshness fix — 2026-10-01

Scoped amr_manipulation build completedexit0 (1.90s). Installed executable
`gate6_product_test` matches source SHA1157a3334f3ef66e1596024cc2dd4f097435fb1c67f154ce4206af7e708d7535.
Fresh recorded GUIrun23 started via `run_full_factory.py --run-id
factory_full_validation_20261001_23 --domain232 --gui` (CLI uses separate
`--domain 232` tokens); authoritative execsession2056. No production edits
while active, original model retained; mandatoryfailurestop and ownedcleanup.
Pre-run allocated logs264,413,184/evidence344,862,720bytes, bothwithin500MB.
Run23 progress: Gazebo GUIclient73356 onDISPLAY:0; host/runtime/graph/lifecycle/
MoveIt/recorder/observer/mode gatesexit0. Startup aggregateRTF0.996327, median
1.000019. Product101gateexit0; Product102nowactive. No fullrunacceptance yet.

gpt-6.1-sol/medium changed only gate6_product_test.py and
test_product_test_contract.py. `_navigate` discards stale terminal tuple before
unchanged precise-only AMCLfallback; `_align_dock_heading` preserves registered
precise closure for yaw<=.15rad, otherwise uses normal heading-only at finite
fresh achievedXY. Both successful initial/recovery travel paths use it;
abort/collision dispatches no new motion. Focused pytest41passedexit0; source
compile/scoped diffcheckexit0. Intermediate old source-string expectation was
replaced with behavior coverage after21pass/1fail; no secondproductionpatch.
Analyzer packet stability_20261001/navigation_packet.md is60lines. No controller
tuning/geometry/dynamics/acceptance threshold change. Next scopedbuild then
fresh original-model GUIrun, firstmandatoryfailurestop; independentreviewafter.

---

## Active task — measured Gazebo stability optimization — 2026-10-01

User reports prolonged pickup/placement oscillation and pickupB overshoot or
overturning. Functional run22 PASS remains valid for its gates, but smoothness
and efficient convergence were not established. Latest GUI run
`visual_factory_20261001_01` CLI exited1 with `Gate 6 child exited with status 2`;
its factory launcher has exited. Orphaned owned MoveIt session43857 was stopped
with Ctrl-C and exited0; no replacement simulation has started.

Additional authorized task: user supplied CAD `amr_v2/` with an arm-mounted
camera and requests replacement of the old model. Found129 MB SolidWorks URDF
export and meshes in workspace; export is a ROS1/catkin package, not a runnable
ROS2 replacement. Medium worker navigation_trace now owns read-only CAD
integration diagnosis. Inspect CAD scale, links/joints, collision/inertial data
and camera frame before integration; preserve current-model baseline metrics
and distinguish controller effects from new geometry in fresh validation.
Do not invent dimensions or camera calibration while waiting for the file.

User explicitly authorizes small delegated tasks: gpt-6.1-sol/medium workers,
gpt-6.1-sol/high analyzer; failures require discussion/re-diagnosis. Agents
stability_metrics, navigation_trace and stability_analyzer are gathering
baseline bag metrics and source/log evidence before any production tuning.
Metrics worker alone owns bounded finalized-bag decoding. Existing safety
gates, tolerances, robot geometry and ownership contracts remain unchanged.

Completed policy task: AGENTS.md now records exact task-specific delegation,
two-minute agent progress, immediate handoff updates after successful tasks,
stability reporting after every simulation and the user's maximum60-line
stability report. It also records separate500,000,000-byte retained log/evidence
caps (logical and allocated, archives and temporary decoding included).
Existing dirty work and protected AMR_CODEX_HANDOFF.md are preserved.
Next: diagnose measured failure, issue a bounded implementation packet, then
fresh comparable Gazebo validation with quantitative stability reporting.

Completed storage task: removed only26 superseded run21 `.db3.zstd` parts,
retaining all reports/metadata (explicitly labeled purged) and full run22 bag.
Exact sizes and SHA256 are in
`phase14_evidence/stability_20261001/superseded_run21_removal.json`.
Retention helper initially hit Python3.10's absent hashlib.file_digest before
any removal; bounded SHA256 streaming corrected it and the run exited0.
Protected handoff SHA remains032bf29994fae1610585a62a0a23d26a23eda6753e78a3fb1513d16bdab5f114.

Completed navigation trace task (gpt-6.1-sol/medium, read-only): GUI Product101
completed; Product102 preparation dock travel ended yaw0.2379rad, then a
same-XY exact-yaw request to rotate-disabled PlacementFollowPath failed its
10s progress check. Recovery hit collision checking and physical dock proof
rejected XY0.0177m/yaw0.2126rad. Run22 ingress already ended yaw0.0710rad,
masking the terminal-yaw capability mismatch. Analyzer is establishing the
safe minimal ingress correction; no tuning/source patch authorized by evidence
yet. Placement logs include deliberate bounded segments and cannot alone prove
closed-loop oscillation. Relevant source: gate6_product_test.py1074–1075,
controller.yaml27–31,85–89; GUI python3_63409 and controller_server logs retained.

Completed ROS2 skill improvement: navigation/MODULE.md now requires quantified
per-action stability, command versus measured response, explicit deadbands,
sim/wall timing, planned-maneuver separation and independent3D roll/pitch.
diagnostics/nav2-failure.md now checks active-controller terminal capability
instead of assuming MPPI tuning. These gaps were exposed by functional PASS
masking the GUI terminal-yaw mismatch and planar odometry omitting tipping.
Changed only these two files under /home/pete/.codex/skills/ros2/ as explicitly
authorized by the user; no robot calibration/settings changed.

CAD diagnosis in progress: whole-assembly STL duplication and continuous
base-fixed camera joint prohibit direct import. New export's left wheel axis
transforms to base-Y while right transforms+Y; current model correctly maps
both positive wheel velocities to forward. Pending user clarification via
text question: camera's intended arm parent and relative mount transform.
Worker is checking exact connected-component extraction against old geometry;
do not invent an arm-camera pose or import mirrored drive axes.

Completed baseline metrics task (gpt-6.1-sol/medium): finalized run22 decoded
with bounded parts, all1,058,268 records verified; both decode passes and cached
recompute exited0, temporary files removed. `stability_20261001/baseline_report.md`
is39lines; baseline_metrics.json and reusable analyze_stability.py retained.
JobsA/B180.40/171.80s; explicit placement alignment39.78/33.44s. All33controller
legs have zero within-leg linear reversals; precise legs have zero angular
reversals. DispatchA has4angular course reversals, all far from endpoint.
Physical quaternion122,818samples maxroll6.41e-9/maxpitch2.20e-8rad demonstrates
no tipping in this baseline. No terminal hunting/overshoot is demonstrated.
GUI run's concrete yaw mismatch remains supported separately; do not use
baseline pass to dismiss it or tune controller parameters speculatively.

Completed storage recompression task: all26run22 FILE-zstd bag parts losslessly
reencoded atzstd19, streaming with decoded SHA256/byte-count identity verified
before atomic replacement; commandexit0. Full baseline remains native-readable.
Receipt is retention_20261001/bag_reencoding_receipt.json. Allocated evidence
348,573,696bytes/logs264,212,480bytes, both below500,000,000;150MBheadroomcheck
passes. No source/robot-function change.

Completed CAD diagnosis task (gpt-6.1-sol/medium): original source untouched.
All54existing chassis components canonically match newCAD at1µm precision;
new export includes baked KUKA arm (CSV) without articulated joints. Preserve
active six-joint arm and known dynamics. New camera components109/110 remain
identity candidates; mount link/configuration/calibration missing. Exact
component extraction and isolated fixes can proceed only with gated mapping,
not wholesale import of whole-assembly geometry or aggregate37kgbase inertia.
Latest user explicitly requests fixing and listing all URDF problems; worker
and high analyzer are preparing a bounded remediation packet.

Completed URDF audit task: `stability_20261001/urdf_audit.md` lists59lines of
export defects versus active-model status, exact verified chassis/LiDAR/caster
correspondence and withheld wheel/camera/aggregate-inertia replacement. No
production model edit yet; safe partial extraction awaits analyzer approval.

Sol/high approved isolated navigation/freshness packet; medium stability_metrics
is sole production writer for gate6_product_test.py and its focused contract
tests. Stale terminal feedback currently survives missing AMCL fallback;
realmethod baseline probe failedexit1. Clear stale pose before unchanged bounded
fallback. When succeeded precise ingress yaw exceeds unchanged.15rad, use
normal angular-capable heading-only goal at achieved fresh localizedXY; inside
.15rad preserve existingfastprecise secondgoal. Apply bothinitial/recovery,
never move after abortedtravelcatch. GUI recovery ingress collision remains
unresolved; fresh recordedrun will discriminate and stop on mandatoryfailure.

Completed disposable shader-cache cleanup from exited visualGUIrun; exact
receipt `stability_20261001/disposable_gui_cache_removal.json` retained. Original
CAD and all functional robot/source/structure remain untouched.

Completed GUI recorder task (gpt-6.1-sol/medium): existing owned full runner
now accepts optional `--gui` (defaultheadless unchanged), records GUI flag,
and changes only Gazebo headlessfalse for that option. All mandatory gates,
topics/recorder/finalproof/teardown/storage checks remain intact. Help/syntax
and generated-source comparison exited0; receipt in
`stability_20261001/gui_runner_validation.md`. Runtime not yet launched.
Next discriminator is fresh original-model GUI recording of pickupB ingress
path, localization/physical bias and velocity response; no production tuning
packet yet. First failed mandatory gate will stop and return to diagnosis.

---

## Latest checkpoint — Gazebo full simulation PASS — 2026-10-01

Fresh actual Gazebo run22/domain232 completed with runner exit0. The command
launched `gz sim` with the factory world, hardware rendering, localization,
Nav2, MoveIt, native attachment, recorder and final observer. Product101,
Product102 and registered home all exited0. All17 mandatory gates passed,
including finalized recorder integrity, both unchanged analyzers and clean
owned teardown with no survivors.

Final observer measured home XY error0.0087645 m, home yaw error0.0086742 rad,
Product101 slot error0.000003690 m and Product102 slot error0.000025911 m.
Fresh empty stow, stationary READY, native detach, fresh bootstrap detach,
factory idle/no-fault and completed_jobs=2 all passed. Run evidence is in
`phase14_evidence/factory_full_validation_20261001_22/`; result.json reports
`success: true`.

Storage after the run remains within the standing caps: logical logs211,857,723
bytes and evidence415,668,738 bytes; allocated logs262,553,600 and evidence
425,840,640 bytes. Both remain below500,000,000 bytes. No active Gazebo or
runner processes remain. Independent source review, hardware and AWS remain
unverified; the simulation result is runtime evidence only.

---

## Latest checkpoint — command references synchronized — 2026-10-01

User explicitly requested full and quick commands in /home/pete/sh&text/amr_cmd.
Updated workspace docs/SIMULATION_COMMANDS.md, that folder's factory quick
reference and its quick index. Added SIMULATION_COMMANDS.md there as a symlink
to the canonical full workspace reference. This explicit command-folder request
authorizes those documentation paths; no other outside-workspace file changed.
Both full/quick instructions contain identical owned run21 workflow commands,
fresh run ID, valid domain232,150 MB pre-launch storage headroom check, all17
mandatory gates and both unchanged analyzers. Manual GUI spawn uses approved
(-4.5,-1.5,0); ordered calls are send101,send102,go home timeout180; readiness
includes normal lifecycle activation and MoveIt. Recorders use FILE-zstd25 MB
splits. Total500,000,000-byte log/evidence caps replace old1 GB instructions.
All58 Bash command blocks parse, setup helper bash -n passes, command blocks
match, full shortcut resolves correctly, headroom check passes. Commands were
not launched; no new simulation or production change. Run21 remains PASS.

User's standing disposable-workspace-removal authorization was used to remove
24 superseded run20 compressed bag parts (64,152,622 bytes), retaining every
report, home collision trace, source/geometry probe and full run18/run21 bags.
Exact receipt retained; metadata is labeled historical/purged. This supersedes
the earlier checkpoint's statement that the full run20 bag remained retained.
Latest allocated logs~261 MB/evidence~346 MB; logical~211/336 MB. Both categories
stay below500 MB and now have enough room for the documented precheck.
Independent source review remains pending; no agents/model changes or commits.
Protected AMR_CODEX_HANDOFF.md and the passing runtime's production source
remain unchanged.

---

## Latest checkpoint — FULL FACTORY SIMULATION PASS — 2026-10-01

Fresh run21/domain232 runner exit0. All17 mandatory gates passed: Products101
and102 then registered home, fresh final physical/empty/native/bootstrap state,
finalized bag integrity, both unchanged strict analyzers and owned teardown.
Factory/MoveIt/recorder/final observer all exit0; no survivors or child crashes.
Physical home error0.0080563 m /0.0016523 rad; final slots after home101=
0.000003699 m and102=0.000028994 m, below unchanged0.030 m. Full report:
phase14_evidence/full_factory_runtime_acceptance_20261001.md. Exact commands,
results, source hashes and complete split lossless bag are in run21 evidence.

Run20 home contact root cause is fixed in factory_supervisor_node.cpp: within
fresh tested dispatch approach envelope, owned normal heading leg, existing
precision translation, then normal registered home heading. Same client owns
exact UUID cancellation and terminal proof. Fresh TF <=300 ms and heading-
phase envelope must hold before further dispatch. Generic home preserves its
normal route. Package adds already-installed tf2_ros build dependency only.
Latest factory build exit0; registered CTest9/9 exit0 including13 behavioral
probes. Scoped diff --check exit0. Prior manipulation source checks and full-
robot geometry discriminators are documented; aggregate tests are not runtime
proof. Independent source review remains pending; author verification is not
independent. Hardware and AWS were excluded and remain unverified here.

User now authorizes workspace removal without further approval when robot
function and structural detail are unaffected. This supersedes prior non-log
bin/confirmation requirements for disposable workspace artifacts. Removed
25 superseded run19 bag parts (65,064,895 bytes) and two generated diagnostic
binaries (12,563,688 bytes), with exact SHA receipts. All reports/traces and full
run18/run20/run21 evidence remain. Stronger bag encoding proved identical decoded
bytes/SHA. No duplicate bin retained. Expanded accounting includes known build
logs and bag artifacts outside the three managed roots. Both total caps remain
500,000,000 bytes, including archives and retained bin content, logical and
allocated usage. Current allocated logs~261 MB/evidence~410 MB; logical~211/
400 MB. Shared utility and final storage snapshot record exact current figures.
Entire workspace is~2.8 GiB; active editor index~1.4 GiB is untouched.

AMR_CODEX_HANDOFF.md SHA unchanged:
032bf29994fae1610585a62a0a23d26a23eda6753e78a3fb1513d16bdab5f114.
No active runtime, agents, model switches, installs, external changes, commits
or pushes. All preexisting dirty work remains. No further runtime rerun is
needed for this unchanged candidate. Safe next step is independent source
review in the authorized workflow; no delegation is presently authorized.

---

## Latest checkpoint — run20 home diagnosed; run21 ready — 2026-10-01

Run20/domain230 exit1 at final slots. Both products, home, all other final
physical/fresh empty/native/bootstrap gates, finalized integrity and owned
teardown passed. Before home both slots were within0.000515 m; normal home
curve first displaced both at robot(-3.4006,0.2279,3.0469). Final errors
0.646151/1.309884 m exceed unchanged0.030 m. Run20 is NOT PASSED. Product102
unchanged analyzer now passes exit0, confirming the public STARTING fix.

Full-robot FCL reproduces the home contact and clears the tested straight
southwest route including tracking/heading envelopes and endpoint turns,
probe exit0. Packet: phase14_evidence/run20_home_route_packet_20261001.md.
Factory supervisor now uses owned normal heading, existing precision travel,
then normal registered home heading from the tested dispatch approach envelope.
Fresh TF <=300 ms selects/rechecks this envelope; any failed leg, cancellation,
stale pose or heading drift stops further dispatch. Same client owns acceptance,
exact UUID cancellation and terminal proof. All public interfaces, slot/home
geometry, thresholds and safety gates remain. Generic home keeps normal route.
Factory package adds already-installed tf2_ros build dependency, no install.

User now authorizes workspace removal without further approval when robot
function and structural detail are unaffected. This supersedes the earlier
per-file non-log/bin confirmation rule for such disposable artifacts. Exactly
25 superseded run19 compressed bag parts were removed (65,064,895 bytes), with
SHA receipt; all its reports/traces and full run18/run20 bags retained. Native
run19/run20 lossless stronger encoding proved decoded SHA and byte equality.
Allocated retained logs~248 MB and evidence~333 MB, logical~200/330 MB, each
below500,000,000. All archives and retained bin content count. Source/robot
structure and AMR_CODEX_HANDOFF.md remain protected; its SHA is unchanged.

Focused build amr_factory exit0; autonomous behavioral probes13 pass before
final heading-envelope guard, registered CTest checks fresh candidate. Next
fresh run21/domain232 uses every original topic and unchanged analyzers,
periodic cap checks and50 MB cleanup reserve. Direct session only, no agents,
model switches, installs, external changes or commits. Stop at first failed
mandatory gate, clean up, diagnose before another edit. Independent review
remains pending; author verification is not independent acceptance.

---

## Latest checkpoint — run19 diagnosed; run20 ready — 2026-10-01

Run19/domain231 runner exit1, mandatory final observer failed. Both product
CLI commands and home exited0; factory, MoveIt and recorder teardown exited0;
no survivors or child crashes. Finalized bag rows/integrity PASS. Offline final
state proves home0.058493 m /0.001984 rad and Product1020.000013 m slot error,
but Product1010.045232 m exceeds unchanged0.030 m. Do NOT call run19 passed.
Product101 remained0.0000037 m from its slot after empty clearance and first
moved during Product102's loaded placement turn at the dock. Full-robot FCL
reproduces that contact and clears the prealigned arrival route. Detailed
source/timing evidence and rejected heading variants are recorded in
`phase14_evidence/run19_remaining_blockers_packet_20261001.md`.

Candidate production changes (gate6_mass_stage.cpp only for this packet):
Product102 establishes projected lateral position at clear dispatch approach,
then follows pi-0.03 into its unchanged0.755/0.100 stance. Registered dock
admission, position reference and every existing bound remain. Remaining
centered corrections use precision directly, failing instead of turning there.
The genuine initial STARTING state remains published until its fresh denied-
motion marker is observed forwarded publicly (8s/cancel fail closed); existing
READY/stationary proof still follows. Standalone stages publishing directly to
public status retain their prior behavior. Timing evidence from run19 shows
internal marker0.599s but public stream misses it; unchanged analyzer102 fails.
No analyzer source/gate or collision/IK/payload checks changed.

The final observer now starts before jobs with actual RELIABLE/VOLATILE native
QoS and retains real attach/detach events. A fresh existing bootstrap service
verification and all unchanged final gates remain mandatory after home. It
waits on an owned file trigger and exits cleanly before teardown; unexpected
observer exits stop the runtime. The earlier final-observer QoS error and two
stale exact source assertions (new aligned call count and derived dock target)
are direct-session mistakes, documented in this packet, not Luna mistakes.

Fresh source verification: focused moveit_config/completion/cycle_adapter
pytest exit0,87 passed; manipulation colcon build exit0; registered CTest
exit0,8/8 with27 action/observation behavioral tests. Runtime helper syntax
and all exact workflow anchors PASS; scoped diff --check exit0. Factory9/9
registered checks remain from the unchanged previous source packet. No source
check or geometry replay is full runtime or independent review acceptance.

Lossless stronger diagnostic archive encoding saved another41,366,104 bytes
with exact decoded SHA parity; receipt retained. Current logical usage:
logs198,008,518 bytes, evidence337,051,788 bytes; allocated usage245,501,952
and339,640,320 bytes respectively. Both total500,000,000-byte caps remain in
force, including archives and retained bin content. Next run20/domain230 uses
25 MB split zstd file bags with bounded decoding, all original topics, unchanged
analyzers, periodic storage checks and50 MB cleanup reserve. Home relocation
and exact purge approval remain in force. No additional runtime permission
is needed. No agents/model switches, commits, installs or outside-workspace
changes. Independent review remains pending; protected handoff unchanged.
Stop at first mandatory runtime failure, cleanup then diagnose before editing.

---

## Latest checkpoint — home and retention approved; fresh runtime next — 2026-10-01

The user approved home relocation and clarified that **all retained logs together
must stay <=500,000,000 bytes and all retained evidence together must stay
<=500,000,000 bytes**. Archives and bin contents count. The user's `+` answered
the exact retention-purge question; the approved manifest and purge receipt are
in `phase14_evidence/retention_20261001/`. This checkpoint supersedes the pending
home/storage decisions below. No additional general runtime permission is needed.
Direct current-session work only; no agents, model switches, installs, external
changes, commits or pushes. Hardware and AWS runtime remain excluded.

Home is now (-4.5,-1.5,0) in the station registry, AMCL initial pose, active
factory launch defaults and canonical commands. Product slots, dimensions,
collision gates and acceptance limits are unchanged. Focused home tests pass
38/38. Projected same-heading reverse replay (including its Y displacement)
passes using the full robot model; registered old home and recorded run18 turn
still reproduce collision. Its first invocation used an incorrect SRDF path
and exited 134 with `missing model file`; corrected existing description path
passes exit 0. No production edit followed that harness-path mistake.

Lossless retention preparation preserved run18 and accepted AWS run05 bag
archives, verified compressed/decompressed SHA256 identity, and retained maps,
reports, source probes and replay snapshots. Superseded raw bags/oversized legacy
archives, generated caches and verified lossless-copy originals were purged
against the approved 621-entry manifest. A newly created disposable compressed
validation copy was also removed after exact parity and both analyzer passes.
Receipt lists all 622 paths. No duplicate bin was retained. Current logical
usage is logs=194,379,030 bytes and evidence=301,906,664 bytes; allocated usage
is logs=241,508,352 and evidence=304,197,632 bytes. These figures cover `.ros_logs`,
`phase14_evidence` and `log`. `.vscode/browse.vc.db` is the editor's generated
C++ index, outside this log/evidence purge scope; it was not modified.

Recording now uses native zstd FILE compression with 25 MB splits, every
previous mandatory topic and unchanged QoS. A task-local bounded reader decodes
one SQLite part at a time, verifies integrity, and passes the unchanged strict
analyzer the same topic/timestamp/payload tuples. Against all 1,015,146 messages
and 61 run18 topics, both MESSAGE and bounded FILE decoders match the original
exactly; both unchanged analyzers pass separately with exit 0. No analyzer
source, topic requirement or acceptance gate changed. The initial finite-queue
MESSAGE conversion dropped rows and was rejected; the original was untouched.
The rejected derivative is included in the approved purge receipt. The runtime
storage monitor counts logical and allocated bytes, stops with 50 MB reserved
for cleanup, and never reads active SQLite evidence.

Fresh source verification:

```bash
ROS_DOMAIN_ID=223 colcon build --packages-select amr_manipulation amr_factory --symlink-install --executor sequential --event-handlers console_direct+
# exit 0; 2 affected packages
ROS_DOMAIN_ID=223 ROS_LOG_DIR=/home/pete/amr_ws/log ctest --test-dir build/amr_manipulation --output-on-failure
# exit 0; 8/8 registered tests
ROS_DOMAIN_ID=223 ROS_LOG_DIR=/home/pete/amr_ws/log ctest --test-dir build/amr_factory --output-on-failure
# exit 0; 9/9 registered tests
# Runtime helper syntax and every exact workflow transformation anchor: PASS
# Scoped diff --check: exit 0; protected handoff SHA unchanged.
```

**Full factory simulation is still NOT PASSED.** Run18 remains the latest
runtime until run19 begins. Next command uses unique run19/domain231, both
products then home. Mandatory acceptance additionally checks physical home
<=0.07 m / <=0.15 rad, both final slots <=0.030 m after home, fresh measured
empty stow, native and fresh bootstrap detach, stationary READY, idle no-fault
factory with completed_jobs=2, finalized bag rows/integrity, both unchanged
analyzers, clean owned launcher/MoveIt/recorder exit 0, and no survivors/crashes.
Teardown signals the owned launcher first, retaining group fallback cleanup.
Stop at the first failed mandatory gate, clean up, and diagnose before another
production edit. Source passes and offline parity are not runtime acceptance.
Independent review remains pending; author verification is not independent.

---

## Latest checkpoint — source changes validated; decisions pending — 2026-10-01

The user explicitly resumed with "Fix all the remaining blocker" and added
the rule that logs and evidence must not exceed **500 MB each**. Work remains
direct in this session; no agents, model switches, installs or external
changes are authorized. This checkpoint supersedes the paused authority below.
No further general resume permission is needed, but the two pending decisions
must be resolved before their dependent work.

**Full factory simulation remains NOT PASSED.** No new factory/Gazebo run has
started. Run18 remains the latest full runtime evidence. Home relocation to
(-4.5,-1.5,0) is still unapproved; the explicit question is pending. A separate
question asks whether 500 MB applies per run or to all retained logs/evidence.
No old evidence has been removed, overwritten, or moved to evade the cap.
Directory usage (`du -sm`) is .ros_logs=12439 MiB, phase14_evidence=4369 MiB,
log=259 MiB. Those directories mix logs, bags, diagnostic JSONL and cache data;
they are not separate log/evidence categories. Existing compressed historical
bags alone exceed 500 MB, so a combined retention cap requires a retention
decision rather than a new directory. Do not start a full run before its
storage budget and safe retention are resolved; keep every mandatory topic
and the unchanged evidence analyzers.

Implemented candidate changes:

- `src/amr_manipulation/src/gate6_mass_stage.cpp`: after measured empty stow,
  existing READY/stationary proof and native detach/30 mm slot proof, reverse
  along the current heading to dispatch approach X using the existing mission
  retreat endpoint. Fresh map/base TF (300 ms) supplies localized coordinates;
  fresh physical pose supplies the registered clearance distance. No normal
  heading goal is added beside delivered products. The owned child cannot
  report success until retreat terminal, unchanged physical pose envelopes,
  stationary/detached and final-slot proofs pass. Lost detach during motion
  cancels the owned UUID and requires terminal proof. Loaded-only dock egress,
  slots, robot dimensions and existing collision/controller gates are unchanged.
- Explicit already-installed `tf2_ros` dependency in manipulation CMake/package
  metadata; the listener uses the existing node executor, with no extra thread.
- `src/amr_factory/launch/factory_localization.launch.py`: factory-owned Gazebo
  process exit callback emits Shutdown only if teardown has not already started.
  Unexpected clean/crashed exits still shut the graph down. Exact installed
  ros_gz helper source and run18 show an unconditional second Shutdown after
  Gazebo exit; its string "false" is truthy. Direct Gazebo argv removes that
  helper's unconditional exit action while retaining Harmonic v8, plugin paths,
  server/GUI arguments and rendering requirements. No installed file changed.
- Focused coverage in `test_gate6_pickup_retreat_behavior.cpp`,
  `test_gate6_completion_contract.py`, `test_moveit_config.py` and
  `test_factory_demo_contract.py`.

Fresh verification:

```bash
cmake --build build/amr_manipulation --parallel 2
# exit 0; production and behavioral binaries built
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_factory/test/test_factory_demo_contract.py src/amr_manipulation/test/test_gate6_completion_contract.py src/amr_manipulation/test/test_moveit_config.py src/amr_manipulation/test/test_cycle_adapter.py
# exit 0; 99 passed, one existing launch warning
ROS_DOMAIN_ID=223 ROS_LOG_DIR=/home/pete/amr_ws/log build/amr_manipulation/gate6_pickup_retreat_behavior_test --ros-args --disable-external-lib-logs
# exit 0; 25 passed, including seven new dispatch-clearance checks
git diff --check -- src/amr_manipulation src/amr_factory
# exit 0
```

The actual launch ROSAdapter was also exercised with a sleeping owned dummy
process, an initial Shutdown and its subsequent exit: old `on_exit=Shutdown()`
reproduces `Cannot shutdown a ROS adapter that is not running`, launcher exit 1;
the production guarded callback exits 0. Probe domain 223; logs under
`log/factory_teardown_contract_20261001/`. Both probe processes finished.
This is an orchestration discriminator, not a factory/Gazebo acceptance run.

Direct-session validation corrections: the first focused suite had 43 passes
and one stale assertion counting exactly three navigation cancellation paths;
the new detached guard requires a fourth. The first assertion update reused a
helper that permits only log/return and rejected the necessary cancel call.
It was replaced with an exact log/cancel/return assertion; final suite passes.
An initial no-op patch anchor was rejected without changing the file. These
are direct-session mistakes, not Luna mistakes; no Luna ledger entry/count
change applies. No production failure was concealed or gate weakened.

Safe next step: resolve home pose and storage scope/retention, finish the home
registry/defaults packet if approved, verify the projected reverse path against
the preserved full-robot geometry, run affected-package colcon/registered
checks, then fresh run19/domain231. Require both unchanged product analyzers,
both final product slots after next-job/home motion, successful home pose,
fresh empty/detached idle/no-fault status, finalized recorder integrity and clean
owned teardown. Source checks do not establish any of those runtime gates.
Independent review remains pending and author verification is not independent.
AMR_CODEX_HANDOFF.md SHA256 remains
032bf29994fae1610585a62a0a23d26a23eda6753e78a3fb1513d16bdab5f114.

---

## Latest checkpoint — USER PAUSED — 2026-10-01

The user requested this handoff before leaving. Work is paused: do not resume
debugging, launch a simulation, delegate, or change production without a new
explicit resume instruction. This checkpoint supersedes older active/next-run
directions below; the earlier entries are retained as history.

**Full factory simulation: NOT PASSED.** Latest authoritative run:
`factory_full_validation_20261001_18`, ROS domain **229**, runner exit **1**.
All startup/preflight gates passed. Product 101 and Product 102 each completed
with CLI exit **0**, and each unchanged bag evidence analyzer separately returned
**PASS / exit 0**. Home returned **exit 1** (`registered home navigation failed`)
after the normal controller rejected a collision ahead. Individual product
passes are not full acceptance: subsequent departure/home motion displaced
delivered products from their slots.

### Remaining blockers and pending user decision

1. Empty-stowed departure turns disturb delivered products. Product 101 began
   leaving its slot during the next job's initial departure; home motion then
   pushed Product 102. Add collision-safe, same-heading reverse clearance before
   any normal next-job/home departure turn, preserving native detach proof,
   fresh state, cancellation/terminal ownership, and final slot proof.
   `/amr/control/dock_egress` is loaded-stow-only: do not widen or reuse that
   contract for an empty departure.
2. Registered home `(-4.5, 0, 0)` physically overlaps delivered Product 102's
   registered slot `(-4.1, 0, 0.075)`. A candidate home `(-4.5, -1.5, 0)` passed
   read-only collision and configured-planner replay checks, but has **not**
   been runtime-validated or authorized. **User approval to relocate home is
   pending**; the leave/handoff request is not approval. Keep product slots,
   robot dimensions, collision checks, and acceptance limits unchanged.
3. Factory launch teardown exits **1** with
   `Cannot shutdown a ROS adapter that is not running`. MoveIt now exits cleanly,
   but the separate launcher/signal-orchestration issue remains unresolved.

Run18 ended with completed_jobs=2, no active job, queue=0, no attachment and
no fault, but that does not establish home or final-slot acceptance. Cleanup
reported no survivors; recorder and bag-info exited 0, and finalized mandatory
topic rows / SQLite integrity checks passed. The authoritative runner and all
probe/build/test handles have finished; no simulation was left active at the
last confirmed cleanup. No further runtime was started for this handoff.

### Implemented and verified so far

- Product 102 stance correction uses the existing release/IK/collision gates;
  run18 completed its full cycle and unchanged strict analyzer.
- Dispatch docking now uses the existing aligned precision route while
  preserving docking admission and placement limits; run18 completed both
  product cycles.
- MoveIt retains its controller-manager plugin with node-local `LD_PRELOAD`,
  preserving existing entries and changing no installed/system files. Paired
  shutdown probes and run18 confirmed clean MoveIt exit; this does not fix the
  separate factory-launch teardown failure.
- Earlier navigation, pickup/localization and bounded placement corrections
  remain in the dirty worktree. No departure/home production patch has been
  made. Author verification is **not** independent review.

Latest focused commands (before the pause):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_manipulation/test/test_moveit_config.py src/amr_manipulation/test/test_gate6_completion_contract.py
# exit 0; 29 passed
colcon build --packages-select amr_manipulation --symlink-install --event-handlers console_direct+
# exit 0
ctest --test-dir build/amr_manipulation --output-on-failure
# exit 0; 8/8 passed
```

Run18 entry point, using the sourced workspace ROS/Gazebo environment and
workspace-local log/temp paths recorded in the runtime evidence:

```bash
python3 phase14_evidence/factory_runtime_tools/run_full_factory.py --run-id factory_full_validation_20261001_18 --domain 229
# exit 1; both product cycles passed, home failed
```

Evidence / exact diagnosis:

- [Departure/home diagnosis](phase14_evidence/dispatch_departure_home_diagnosis_20261001.md)
- [Run18 artifacts](phase14_evidence/factory_full_validation_20261001_18/)
- [Product 102 collision packet](phase14_evidence/product102_collision_packet_20261001.md)
- [Dispatch precision packet](phase14_evidence/dispatch_precision_dock_packet_20261001.md)
- [MoveIt shutdown packet](phase14_evidence/moveit_shutdown_packet_20261001.md)

### Safe resume point

After explicit resume, obtain the unresolved home-placement decision before
changing that contract. Re-read current instructions, this checkpoint and the
departure/home diagnosis; inspect `git status --short` and preserve all dirty
work. Resolve a narrow empty-stowed reverse-clearance packet from the recorded
runtime geometry, then focused validation before another full run. Diagnose
launcher teardown separately; the launcher-only SIGINT versus process-group
signal hypothesis is not yet tested for the factory launcher.

Next proposed fresh run: `factory_full_validation_20261001_19`, domain **231**
(all commands must remain within domain 0–232). Acceptance requires both
unchanged product analyzers, successful home action/pose, both products still
within their unchanged final 3D slot tolerance after the next cycle/home,
fresh detached/empty-stowed proof, completed_jobs=2 and idle/no-fault state,
finalized recorder integrity, and clean owned-process teardown with no
survivors or child crashes. Build/unit totals and offline replay are not
runtime acceptance. Hardware and AWS remain out of scope/deferred.

All changes stay inside `/home/pete/amr_ws`; retain project files/evidence and
use a workspace bin for any later non-log removal. No agents, Luna, model
switches, commits, pushes, installs or system changes are authorized.
`AMR_CODEX_HANDOFF.md` remains untouched (SHA256
`032bf29994fae1610585a62a0a23d26a23eda6753e78a3fb1513d16bdab5f114`).
`LUNA_IMPLEMENTATION_LEDGER.md` remains at count 25; no Luna was used for these
direct-session corrections. Pending independent review is not claimed complete.

---

**Updated:** 2026-10-01
**Default analysis/review:** GPT-6.1 Sol/high (`gpt-6.1-sol`, high reasoning).
**Default implementation:** GPT-5.6 Luna/max (`gpt-5.6-luna`, max reasoning).

**Default model policy:** `Sol/high` means `gpt-6.1-sol` with high
reasoning effort, and `Luna/max` means `gpt-5.6-luna` with max reasoning effort.
The user directed removal of the conflicting GPT-6 restriction on 2026-10-01.
There is no blanket GPT-6 prohibition or mandatory Luna handoff when the user
explicitly authorizes work in the current selected-model session. The existing
current-session authorization for factory diagnosis, implementation, and
validation therefore remains in force, without Luna or delegation. The exact
runtime model identifier is not exposed by available tools; do not claim it
has been verified. This policy edit does not switch models or change settings.
Under the default workflow, Sol diagnoses/reviews and Luna implements. Verify the
exact model identifier and reasoning effort before any model switch or
delegation; if the assigned model is unavailable, stop
and report the blocker rather than substituting another model family.
The user's no-agent instruction remains in force; use a manual Luna/max
handoff under the default workflow, or work directly when explicitly directed.
This policy update adds no new runtime authority; the previously approved
factory validation scope is unchanged. AWS
testing remains deferred; the factory pickup-position blocker remains open.
All safety gates, debugging limits, stop conditions, and independent-review
requirements remain binding; an author's own verification is not independent
review. No production code or runtime was changed by this policy edit.
Every Luna implementation mistake must be recorded in
`LUNA_IMPLEMENTATION_LEDGER.md` before the packet is closed or handed off,
including its cause, evidence, and exact failure text when available.

### Current task directive — full factory simulation — 2026-10-01

The user explicitly directed persistence until the full factory simulation
succeeds, including any simulation blocker beyond Products 101/102. For this
active task only, the user overrides implementation-attempt/hypothesis caps;
counts and evidence remain recorded, and unchanged retries are not progress.
Work stays direct in the current session without Luna/delegation. Hardware
and AWS remain excluded. Every runtime run stops at its first mandatory gate,
then cleanup and diagnosis precede another source edit. Public interfaces,
fail-closed ownership, safety thresholds, and independent-review requirements
are unchanged. This directive supersedes the cap-based stop in the older
checkpoint below.

The bounded post-turn AMCL observation packet is implemented. Candidate
source contracts passed 23 tests, its three new behavioral checks passed,
the manipulation build exited 0, and registered CTest passed 8/8. Full runtime
success remains unverified. Exact commands, negative behavior, and the
direct-session test-update omission are recorded in
`phase14_evidence/amcl_observation_packet_20260930.md`.
Next run: `factory_full_validation_20261001_13`, ROS domain 224, Product 101,
Product 102, then home, with finalized recorder row/integrity checks.

Run13 ended at dispatch placement alignment, Product 101 exit 1; Product 102
and home were not reached. Pickup admission passed unchanged (AMCL XY 0.009 m,
yaw 0.148 rad), and cleanup/recorder integrity passed with no survivors.
Diagnosis/next bounded precision-translation packet is recorded in
`phase14_evidence/placement_precision_packet_20261001.md`. No source acceptance
is inferred from the passing pickup boundary. The user also requires all
further file changes inside this workspace; any removed non-log file goes to
a workspace bin for later confirmation. Relevant evidence is retained.

Run14 completed Product 101, then stopped at Product 102's first lateral
placement translation. The corrected precision route confirmed Product 101
alignment, but keeping dock yaw for lateral travel was an incomplete
implementation. The current correction adds same-position normal heading
alignment before precision travel when needed, computes closest forward/reverse
travel yaw with fresh localization bias, and keeps the final dispatch heading
and all bounds unchanged. Source contracts passed 25 tests; build exited 0;
three new behavioral tests and manipulation CTest 8/8 passed. Next run15 uses
domain 226. Run14 cleanup had no survivors and recorder integrity passed;
there is still no full factory acceptance or independent source-review claim.

Run15 completed Product 101 and Product 102's corrected lateral navigation,
then stopped at `placement target was outside the deterministic IK envelope`.
Recorded settled release radius was 0.7886363275 m, exceeding the unchanged
0.785 m gate; home was not reached. Cleanup had no survivors and finalized
recorder integrity passed. Read-only reconstruction traced this to Product
102's old 0.085 m coarse-route lead and coarse convergence, not yaw or IK
solver failure. The narrow correction targets the existing physical stance
directly and requires Product 102 to converge with the existing precision
route's 0.01 m tolerance. Physical acceptance and every arm/release gate are
unchanged. Source contracts passed 26 tests; build exited 0; manipulation
CTest passed 8/8. Full packet and direct-session test-update omission:
`phase14_evidence/product102_reach_packet_20261001.md`. Next full run16 uses
domain 227, both products then home. The user explicitly reaffirmed direct
current-session work only (6.1 Sol/xhigh), no agents/model switches, and no
stopping before full simulation success; file changes stay in the workspace.

Run16 completed Product 101, then passed Product 102 navigation, radius/IK,
pre-place execution and payload proof before a colliding exact release goal
failed higher-mass lowering. Cleanup left no survivors; recorder integrity
passed. MoveIt exited -11 during later SIGINT teardown, recorded separately.
Exact current-model replay reproduced the fixed lidar/base contacts and
falsified a seed-only correction. A stance grounded in the earlier accepted
achieved pose, then checked with loaded-scene release/full-L path at nine
nearby positions, passed baseline-red/candidate-green replay. Product 102's
only new production change is stance constants 0.775/0.075 -> 0.755/0.100;
all slots, orientation, motion/safety/arm gates and Product 101 stay unchanged.
Focused contracts 27 passed, build exit 0, manipulation CTest 8/8 passed.
Packet: phase14_evidence/product102_collision_packet_20261001.md.
Next fresh full run17, domain 228, both products then home. Full acceptance
remains unverified until runtime and unchanged per-product analyzers pass.

Run17 stopped before Product 102: Product 101's coarse dispatch arrival passed
the 0.155 m dock gate but left required alignment just above the unchanged
0.35 m cap. Exact finalized-bag poses support a route/admission mismatch;
Product 102 correction remains runtime untested. Dispatch docking now uses
the existing heading-aligned precision route, with the same bias-corrected
registered target and every physical/motion gate unchanged. Build exit 0,
focused contracts 28 passed, CTest 8/8 passed. Packet:
phase14_evidence/dispatch_precision_dock_packet_20261001.md.

Separate MoveIt shutdown diagnosis falsified double-SIGINT as the cause.
Crash address maps to a controller plugin unloaded before retained callback
destruction. MoveIt launch now keeps that library loaded for only its process;
the actual project-launch shutdown probe exits cleanly without external env
override. No outside files, dependencies or global settings changed. Focused
contracts now 29 passed. Packet: phase14_evidence/moveit_shutdown_packet_20261001.md.
Next fresh full run18, domain 229, both products then home, then unchanged
per-product finalized-bag analyzers and final-state proof. No full acceptance
or independent source-review claim yet.

Run18 (domain 229) completed both Product 101/102, and unchanged per-product
bag analyzers each PASS. MoveIt now exits cleanly. Home failed at a controller
collision gate, cleanup leaves no survivors, recorder integrity passes.
Full simulation is NOT accepted: the next job's departure had already pushed
Product 101, and home departure pushed Product 102. Normal in-place/forward
turning beside low delivered products is not guarded by lidar height. Also
registered home (-4.5,0) physically overlaps Product 102's required slot.
Detailed trace: phase14_evidence/dispatch_departure_home_diagnosis_20261001.md.
No departure/home production edit yet. Asynchronous user question requests
approval to relocate simulation home; absent reply is not approval. Continue
read-only geometry/model/retreat diagnosis; preserve gates/slots/dimensions.

Read-only follow-up confirms full-robot contact at old home and recorded
departure. Candidate home (-4.5,-1.5) is collision-free across all existing
XY/yaw tolerance corners; same-heading reverse clearance to dispatch approach
X then turning there is clear. Installed Smac/configured-smoother candidate
replay on exact run18 raw costmap passes, exit 0. These are not runtime proof.
No departure/home production edit made; awaiting explicit answer to home
relocation question because the current exact phase registry contract would
change. Safe resume: approved home/empty-stow clearance packet, focused tests,
fresh run19 domain 231, both analyzers AND both final product poses after home.
Factory teardown's `Cannot shutdown a ROS adapter that is not running` also
remains separately unverified; MoveIt teardown is fixed. All runtime/probe
handles ended and no survivors remain. Protected handoff untouched.

### Historical stop checkpoint — heading loop removed; post-turn AMCL admission race — 2026-09-30

The current user explicitly directed direct implementation in their selected
session, without Luna or delegation, and reports the picker as GPT-6.1
Sol/xhigh. That task-specific direction supersedes the standing role policy
above for this active task only; no model switch, delegation, or global
policy/settings change was made. The exact runtime model ID is not exposed
by available tools. The objective is full factory simulation success for
Product 101, Product 102 and home; hardware and AWS remain excluded.

Current direct-session source changes are confined to
`src/amr_mission/src/mission_supervisor_node.cpp` and its behavioral test:
normal goals already within the unchanged 0.07 m XY tolerance select the
existing collision-checked PrecisionGridBased planner while retaining
FollowPath, goal_checker, public endpoints, external smoothing, and all
reservation/cancellation/terminal ownership. Farther goals and missing TF
keep GridBased. Precise/retreat routing and all thresholds are unchanged.
The real-action regression failed for the intended two near-position cases
against the pre-fix source, then passed after the correction. Build exited 0;
mission CTest passed 3/3; related mission/manipulation contracts passed 29
tests; diff check passed. None of those totals proves factory acceptance.

Run11 (`factory_full_validation_20260930_11`, domain 221) was stopped before
the mass-stage child after a mandatory lidar recorder QoS incompatibility.
It is non-diagnostic for the source fix. The task-owned recorder override
was repaired to documented QoS and a read-only actual-message preflight
was added. No production publisher or gate changed.

Run12 (`factory_full_validation_20260930_12`, domain 222) passed host, timing,
graph, lifecycle, MoveIt and recorder preflights. Product 101 pickup retreat
and the corrected heading action succeeded. The heading path was 0.00984 m
instead of run10's 2.9952 m loop; its terminal localized errors were 0.009 m
XY and 0.142 rad yaw, elapsed 1.453 simulation seconds. The independent
AMCL admission still failed immediately afterward. Its latest bag-observed
AMCL sample was 0.26686 s old with 0.00905 m XY error but 0.20482 rad yaw;
the next sample, yaw 0.14120 rad, arrived only 0.03223 s after the check.
This supports a cached AMCL post-turn observation race, not wider tolerances.
Private callback receipt timing is not instrumented.

Product 101 command exited 1. Product 102/home were not reached. Final
factory status was attached=true, fault_latched=true, completed_jobs=0.
Every owned process stopped; survivors were empty, recorder and bag-info
exited 0, and SQLite quick_check was `ok`. All raw evidence is preserved.
There is no full factory success or independent source-review acceptance.

Two implementation attempts for the pickup-heading admission blocker have
now run. Autonomous production editing stops at the two-attempt cap; user
direction is required for the next packet. Safe resume: an explicit packet
to wait on post-heading AMCL observation within a bounded deadline while
preserving every current gate, with delayed-message baseline/negative tests
and fresh runtime evidence. Do not replace AMCL proof with TF/ground truth,
widen tolerances/freshness, add motion to force observation, or retry hoping
for a pass. Run09's separate empty-stow cache-freshness failure is also still
unresolved. Full diagnosis, commands, timings and proposed next packet:
`phase14_evidence/heading_routing_diagnosis_20260930.md`.

Unrelated dirty work and prior changes are preserved.
`AMR_CODEX_HANDOFF.md` remains untouched. No Luna was involved in these
direct-session changes; no direct-session mistake is attributed to Luna.

### Earlier stop checkpoint — precision docking repaired; pickup-station terminal proof failed — 2026-09-30

The user explicitly authorized direct work in the manually selected Astra/high
session, with a one-time exception to the then-current GPT-6 prohibition and
no agents. That historical exception applied only to the docking task; future
work follows the updated standing model policy above. The user explicitly
requested the ROS 2 skill, which guided the
Nav2 boundary checks and fresh simulation validation.

The Run04 docking collision is reproduced against the recorded geometry.
Navfn appended an exact goal `(2.400,3.000,0.049)` after the grid-aligned pose
`(2.350,2.950,0)`. With the existing `w_smooth=0.0`, SimpleSmoother preserved
those positions but recomputed the penultimate body heading as 45 degrees;
the footprint then reached lethal cost 254. The unsmoothed headings were
clear. A diagnostic `w_smooth=0.3` still collided and was not implemented.
The exact straight approach, including swept heading changes, was clear.
Run04 lacked a raw costmap: the diagnostic inversion preserves lethal and
unknown classes exactly, while ordinary inflation costs are upper bucket
bounds. This limitation is recorded with the replay evidence.

The implementation is confined to `src/amr_navigation`: a
`PrecisionNavfnPlanner` plugin prefers exact straight segments only after
checking the center, rectangular footprint, and endpoint rotation sweeps at
half-cell spacing. It retains the nearest forward/reverse body heading and
the exact requested endpoint; blocked direct segments fall back to existing
Navfn. The existing `PrecisionGridBased` ID selects this wrapper. Normal Smac
configuration, collision thresholds, external smoother, controller, mission
ownership/cancellation, hardware values, and manipulation source are unchanged
by this task. Plugin registration/build dependencies and focused regression
coverage were added. The first build caught a missing Footprint-type header;
the required include was corrected before successful validation.

Fresh source evidence (all final commands exited 0):

```text
colcon build --packages-select amr_navigation --symlink-install --event-handlers console_direct+
ctest --test-dir build/amr_navigation --output-on-failure
  3/3 passed
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_navigation/test/test_navigation_contract.py
  6 passed
build/amr_navigation/test/replay/precision_planner_contract_test \
  .ros_logs/factory_docking_diagnosis_20260930_01/snapshot.json \
  .ros_logs/factory_docking_diagnosis_20260930_01/precision_replay.json
  baseline collision reproduced; candidate collision-free (worst cost 253),
  exact endpoint, reverse path, safety negatives, and plugin load passed
git diff --check -- src/amr_navigation
```

These ROS checks used valid domain 223 and a run-local log directory. CMake
reported the existing non-fatal TF overlay RPATH-cycle warnings.

The first fresh factory run, `factory_docking_20260930_01` (domain 222), passed
preflights but was stopped before Product 101 by a diagnostic-wrapper handle
collision: the completed MoveIt preflight replaced the live launch handle.
The wrapper was corrected after read-only diagnosis; no production edit or
product attempt followed from that non-diagnostic failure. Cleanup and bag
finalization succeeded, with no survivors.

The corrected run was `factory_docking_20260930_02`, `ROS_DOMAIN_ID=221`,
`GZ_PARTITION=amr_factory_docking_20260930_02`. Host, runtime, graph, lifecycle,
and MoveIt preflights passed. Runtime median RTF was approximately 0.9999 and
aggregate RTF 0.9769. The planner server loaded the new precision plugin.
Precise docking succeeded at localized `(2.3906,2.9990,0.0825)`; the cycle
proceeded through manipulation, attachment, loaded stow, 0.502 m dock egress,
and 0.401 m pickup-approach reverse without a smoother/controller abort.

The first mandatory failure was subsequently:

```text
1790761544.269059086 GATE 6 1.0 KG: FAIL:
  fresh AMCL pickup station terminal pose was unavailable
factory_cli.py send pickup_a dispatch --timeout 240: exit 1
```

The last recorded AMCL sample before the failure was about 0.4335 s old,
frame `map`, pose `(1.530588,2.927892,0.082540)`: approximately 0.07833 m from
the registered `(1.5,3.0,0)` pickup approach, exceeding the unchanged 0.07 m
terminal position gate. Recorded ground truth was
`(1.463110,2.921720,0.082540)`. This supports a position/egress-geometry
diagnostic direction rather than merely increasing a freshness timeout; the
exact in-process rejection still requires a focused new diagnosis.

Final factory evidence was `completed_jobs=0`, `completed_cycles=0`,
`active=false`, `product_attached=true`, `fault_latched=true`; detail was
`product cycle failure ended without a fresh empty-stow proof`. The run stopped
and all owned processes exited. The finalized bag is 43.1 MiB, 122276 messages,
81.290186810 s; `ros2 bag info` exited 0 and SQLite quick_check returned `ok`.
It includes 57 raw global-costmap rows and precise action success evidence.
The diagnostic recorder has zero `/map` rows and lacked the documented joint
state QoS override, so this bag is not full Gate 6 acceptance evidence. The
MoveIt segmentation fault occurred after SIGINT during cleanup and is
secondary to the recorded terminal-pose failure.

Evidence and runnable diagnostic artifacts are in
`.ros_logs/factory_docking_diagnosis_20260930_01/` and
`.ros_logs/factory_docking_20260930_02/`. The docking fix has source/replay and
fresh docking-runtime evidence; full factory success, loaded Nav2 retreat,
dispatch, and independent Sol/high review remain unaccepted/unverified.
Safe resume: a new focused diagnosis of the post-reverse pickup-station
terminal-position proof before any further source edit or runtime attempt.
No manipulation/controller/collision relaxation or AWS run was performed.
`AMR_CODEX_HANDOFF.md` is untouched, and unrelated dirty work is preserved.

### Latest stop checkpoint — Factory Product 101 docking runtime — 2026-09-29

The approved factory-docking implementation packets were source-validated and
the final runtime boundary was exercised once under the user's explicit
instruction to stop if it failed. AWS simulation was not rerun.

Source state before the runtime attempt:

- `planner.yaml` contains the new `PrecisionGridBased` Navfn planner while the
  accepted Smac `GridBased` safety configuration is preserved.
- Mission planner routing and planner-ID reservation/cleanup cover normal,
  precise, and registered-retreat endpoints.
- Manipulation launch/supervisor failure propagation and retained-product
  proof are fail-closed; post-reverse pickup-station admission uses the
  existing fresh-AMCL terminal proof without an extra retreat action.
- Focused contracts passed (`24 passed`), the affected package build exited 0,
  and the registered manipulation CTest suite passed `7/7` before runtime.

Final runtime identity:

```text
run_id=factory_docking_20260929_04
ROS_DOMAIN_ID=224
GZ_PARTITION=amr_factory_docking_20260929_04
```

Runtime and graph/lifecycle/MoveIt preflights all passed. The single
`pickup_a dispatch` cycle then stopped at the precise docking gate. The
external smoother rejected the path as colliding at
`(2.350000, 2.950000, 0.785398)`; the mission reported
`path smoothing reached a collision boundary`, and the precise endpoint to
`(2.400, 3.000, 0.049)` failed. The CLI returned 1 after the Gate 6 child
returned 2. No manipulation/grasp/attachment phase began.

Factory status at failure:

```text
completed_jobs=0 completed_cycles=0 attached=False fault_latched=False
detail=Gate 6 child exited with status 2
```

Evidence is preserved in
`.ros_logs/factory_docking_20260929_04/`, including the finalized
`product101_evidence` bag (76.4 MiB, 182346 messages, 117.190949706 s) and
the runtime preflight evidence. The primary failure is recorded in
`smoother_server_64670_1790621137523.log` and
`mission_supervisor_node_64798_1790621145531.log`. All run-owned processes
were stopped and the survivor audit was empty. The MoveIt SIGINT-time
segmentation fault occurred during operator cleanup after the primary failure
and is non-causal.

Per the user's stop instruction, no source edit, retry, or planner/controller/
collision relaxation follows this failed runtime. The safe resume point is a
new Sol/high diagnosis of the smoother collision boundary before any further
implementation packet. Do not claim factory runtime acceptance or terminal
factory success. The required post-runtime independent review is deferred
because the mandatory runtime gate failed. `AMR_CODEX_HANDOFF.md` remains
untouched, AWS remains untouched, and unrelated dirty/untracked work is
preserved. No new Luna ledger entry is warranted by this runtime failure.

## Objective

Finish portable AWS warehouse exploration and obtain a fresh run that reaches
the existing `COMPLETE` gate. Preserve fail-closed behavior, ownership
boundaries, public interfaces, safety gates, and documented hardware values.
Keep `AMR_CODEX_HANDOFF.md` untouched.

### Required completion goal — exploration only

The only remaining work required to finish exploration is:

1. Diagnose and fix the route failures causing smoother/controller collision
   aborts.
2. Diagnose and fix the base/manipulator authority freshness loss.
3. Rebuild and run focused validation.
4. Obtain one uninterrupted AWS run reaching `COMPLETE` with zero unresolved
   frontiers, all motion stopped, no active/pending/cancel-owned goals, no
   latched fault, valid finalized bag/map evidence, and clean process cleanup.

These observed failures may share one underlying root cause; do not assume
three separate fixes. Do not weaken collision, controller, ownership, or
authority-freshness gates.

Optional work is deferred and is not needed for now: hardware acceptance,
human map-quality review, unrelated robot-product phases, and additional
cleanup outside the four goals above.

### Latest stop checkpoint — post-patch AWS runtime — 2026-09-26

The lost arm-goal-response startup failure was reproduced twice in
`aws_warehouse_runner_20260926_01` (ROS domain 219) and
`aws_warehouse_runner_20260926_02` (ROS domain 218). The arm controller
accepted the trajectory, lost the goal response, and the portable stow
authority faulted on the recovered non-terminal result. The narrow correction
was implemented only in
`src/amr_simulation/scripts/portable_stow_authority.py` and
`src/amr_simulation/test/test_portable_stow_authority.py`: UUID recovery now
continues polling explicit `ACCEPTED`, `EXECUTING`, and `CANCELING` statuses
without resending the trajectory; `UNKNOWN`, `CANCELED`, `ABORTED`, rejected,
exceptional, and non-success results remain fail-closed faults. Motion remains
denied while the terminal result is unproven.

Validation after the correction:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test/test_portable_stow_authority.py
24 passed

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test
117 passed, 1 warning

source /opt/ros/humble/setup.bash && source install/setup.bash && \
colcon build --packages-select amr_simulation --symlink-install \
  --event-handlers console_direct+
exit 0

ctest --test-dir build/amr_simulation --output-on-failure
12/12 passed
```

The fresh post-correction runs all passed the arm stow/readiness boundary but
did not reach AWS `COMPLETE`:

* `aws_warehouse_runner_20260926_03` (domain 217): Nav2 smoother rejected a
  path after reporting a collision at `(3.345946, -9.511901, 0.041469)`;
  terminal `SMOOTHER_ABORT`, `FAULT`, finalized bag, no survivors.
* `aws_warehouse_runner_20260926_04` (domain 216): Regulated Pure Pursuit
  reported `detected collision ahead`; terminal `CONTROLLER_ABORT`, `FAULT`,
  finalized bag, no survivors.
* `aws_warehouse_runner_20260926_05` (domain 215): Explorer lost fresh base /
  manipulator motion authority, canceled the active path, and ended
  `CANCELLATION`, `FAULT`; finalized bag, no survivors. Cleanup escalated.

No source change was made for these three failures. Smoother, controller, and
authority freshness gates remain fail-closed and their thresholds were not
weakened. The prior handoff acceptance
`aws_warehouse_runner_20260925_05` remains the existing fresh `COMPLETE`
evidence, but no post-correction `COMPLETE` run was obtained. The next safe
resume requires a new diagnosis and explicitly scoped packet for route /
controller robustness or authority-freshness stability. `AMR_CODEX_HANDOFF.md`
remains untouched.

### Latest stop checkpoint — Packet 6 raster-fixture diagnosis — 2026-09-25

GPT-5.6 Luna/max attempted the approved Packet 6 replay-coverage packet and
stopped after two failed attempts on the same swept-rotation fixture
hypothesis. Baseline navigation CTest passed 2/2, the preserved Run07 replay
passed all six cases, and the replay harness built. The focused
`planner_replay_contract` failed twice because the supposedly clear endpoint
returned lethal cost 254.

GPT-5.6 Sol/high re-diagnosed the failure and found the supported root cause:
continuous fixture geometry did not match Nav2's 5 cm rasterization. The
obstacle at `(0.425, -0.425)` maps onto the yaw-zero endpoint footprint edge
at cell `(48,31)`. An independently checked replacement cell at
`(0.675, 0.175)` is clear at both endpoints and lethal during the interpolated
rotation at approximately `0.735266366` rad. The existing between-pose
interpolation and endpoint-compatibility reporting are present in
`src/amr_navigation/test/replay/planner_replay_backend.cpp`; no route fixtures
or CTest route registration were added. The replay directory is untracked, so
the partial implementation must be preserved and inspected rather than
reconstructed from Git.

The safe resume point is one new GPT-5.6 Luna/max correction packet limited to
`src/amr_navigation/test/replay/planner_replay_contract_test.cpp`: replace only
the invalid obstacle coordinate, validate the focused contract, full
navigation CTest, and the six-case Run07 replay, then obtain independent
GPT-5.6 Sol/high review. Do not add route fixtures, change thresholds, or
claim AWS completion in that packet. The two prior implementation attempts
remain consumed; this raster-aware diagnosis establishes a new hypothesis.
The ledger currently records 7 mistakes. No new implementation was started
after this diagnosis.

No AWS completion claim is made. `AMR_CODEX_HANDOFF.md` remains untouched.

### Authorized implementation packet — Safe Reachable-Area Completion v1 — 2026-09-25

The user explicitly authorized a new completion policy for the Run04 evidence.
This section supersedes the prior safe-resume requirement below that asked for
policy authorization before source work.

`COMPLETE` now means that no frontier remains safely reachable under the
current validated map, raw costmap, padded rectangular footprint, and existing
heading-aware route proof. It does not claim that every raw frontier or
unknown cell is physically accessible. After three fresh, content-matched
zero-candidate planning cycles, each raw frontier must be classified as
`BLOCKED_SAFETY` or `BLOCKED_ROUTE`; an unclassified frontier remains
fail-closed `INCOMPLETE`. Terminal proof still requires no active, pending, or
cancel-owned motion, `motion_stopped=true`, and no latched fault. Planner,
inflation, footprint, unknown-space, smoother, controller, and collision
thresholds are unchanged.

The implementation adds optional endpoint/route diagnostics to the pure
frontier selector, publishes policy and raw/blocked/unresolved frontier
counts, changes only the validated no-safe-reachable-frontier terminal to
policy `COMPLETE`, and requires the AWS monitor/runner to validate the count
invariants. The existing list-returning selector seam remains compatible.
Unavailable/rejected actions, stale or malformed evidence, smoother,
controller, cancellation, ownership, and other navigation faults remain
fail-closed.

Fresh validation after the implementation:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_exploration/test
195 passed, 2 xfailed, 1 xpassed

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test/test_aws_exploration_monitor.py \
  src/amr_simulation/test/test_aws_exploration_runner.py \
  src/amr_simulation/test/test_aws_exploration_diagnostics.py
38 passed

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test/test_aws_exploration_runner.py
19 passed

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  src/amr_exploration/scripts/frontier_algorithm.py \
  src/amr_exploration/scripts/frontier_explorer.py \
  src/amr_simulation/scripts/aws_exploration_monitor.py \
  src/amr_simulation/scripts/aws_exploration_runner.py
exit 0

source /opt/ros/humble/setup.bash && source install/setup.bash && \
colcon build --packages-select amr_exploration amr_simulation \
  --symlink-install --event-handlers console_direct+
exit 0

ctest --test-dir build/amr_exploration --output-on-failure
3/3 passed

ctest --test-dir build/amr_simulation --output-on-failure
12/12 passed after correcting the stale world-size/hash literals in
test_aws_warehouse_exploration.py; the canonical tracked and installed world
remains 7950 bytes with SHA-256
41d9685cf8104a5e76ea832fa5330eadcbc4f7952c3ed2115f38a352d695fbb8.

git diff --check -- \
  src/amr_exploration/scripts/frontier_algorithm.py \
  src/amr_exploration/scripts/frontier_explorer.py \
  src/amr_exploration/test/test_frontier_algorithm.py \
  src/amr_exploration/test/test_frontier_contract.py \
  src/amr_exploration/test/test_frontier_lifecycle.py
exit 0
```

Fresh runtime acceptance — Run05 — used the maintained AWS runner with
`ROS_DOMAIN_ID=220`, run identity `aws_warehouse_runner_20260925_05`, and a
unique `GZ_PARTITION`. The finalized result is `COMPLETE` with `pass: true`
under `SAFE_REACHABLE_AREA_V1`: 110 raw frontiers, 109 `BLOCKED_SAFETY`, 1
`BLOCKED_ROUTE`, and 0 unresolved. Terminal evidence proves
`active=false`, `pending=false`, `cancel_owned_motion=false`,
`motion_stopped=true`, `fault_latched=false`, and mission outcome `SUCCEEDED`.

The runner finalized one SQLite bag with 151737 messages over 163.600086661 s;
map, raw global/local costmap, TF, exploration status, navigation action,
sensor, odometry, and simulation-diagnostics topics have nonzero rows. The
map saver exited 0 and verified both the run-local YAML and PGM. Recorder,
monitor, and diagnostics exited 0; the launch process returned 1 only during
the terminal shutdown sequence, with no recorded first failure. All owned
processes exited and the survivor list is empty. Final simulation diagnostics
recorded 24/24 monitored collision classes covered and only
`NORMAL_SUPPORT` contacts.

Independent Sol/high review after the runtime found no material policy
finding. The review confirmed that the historical list-returning selector
contract remains compatible, endpoint/route classification uses the existing
safety gates, unresolved diagnostics remain fail-closed, and the AWS
monitor/runner reject `COMPLETE` unless every raw frontier is accounted for.
This is simulation acceptance evidence, not hardware acceptance or a claim of
human map quality. `AMR_CODEX_HANDOFF.md` remains untouched and all unrelated
dirty work is preserved.

### Whole planned AWS simulation closeout — Packet 6 — 2026-09-25

The remaining planned AWS boundary was the planner/footprint replay gate. The
standalone replay contract is now integrated into
`src/amr_navigation/CMakeLists.txt`; no planner threshold, inflation,
footprint, unknown-space, smoother, or controller safety rule was weakened.

Fresh replay evidence used the preserved Run07 snapshot and the installed
Humble Smac lattice primitives. The six-case regression passed: Smac 2D is
retained as the expected unsafe comparison (return 2 with footprint cost 254),
the configured Smac lattice returned a collision-free path through the
external `SimpleSmoother` with worst footprint cost 253, and blocked and
unknown start/goal cases were rejected. The replay CMake build and standalone
contract CTest passed; the integrated navigation package CTest is now 2/2.

The combined `colcon build --packages-select amr_navigation amr_exploration
amr_simulation --symlink-install --event-handlers console_direct+` exited 0,
navigation CTest exited 0, and scoped `git diff --check` exited 0. CMake
reported non-fatal overlay RPATH-cycle warnings for the replay binaries; the
built binaries and tests executed successfully. Together with Run05, this
completes the planned AWS warehouse simulation implementation and acceptance
scope. It does not claim hardware acceptance, full physical accessibility, or
completion of unrelated robot-product phases.

A combined invocation of the Explorer and AWS pytest scopes was non-diagnostic:
after 230 passing tests it segfaulted in runner subprocess cleanup and left two
test-owned descendants. An exact-process audit found those descendants tied to
the temporary cleanup tests; they exited before cleanup, and the isolated
scope runs plus CTest runner case passed. No production patch was made for
that test-order/resource symptom.

### Final AWS software-contract correction — 2026-09-25

The remaining simulation-package failure was diagnosed before editing. The
tracked source world and installed world were byte-identical at 7950 bytes and
the same SHA-256, while the test expected 7954 bytes and a hash that did not
match any current fixture. The production `validate_world` path and all world
semantic invariants passed independently. The smallest correction updated only
the two stale literals in
`src/amr_simulation/test/test_aws_warehouse_exploration.py`; the world file,
planner thresholds, footprint, inflation, unknown-space, smoother, controller,
and runtime safety behavior were not changed.

The corrected AWS contract file passes 8/8 and the registered simulation CTest
suite passes 12/12. The navigation and exploration suites pass 2/2 and 3/3;
the scoped rebuild of `amr_navigation`, `amr_exploration`, and
`amr_simulation` exits 0. The six-case planner replay passes when run serially
with `ROS_DOMAIN_ID=0`. One earlier concurrent CTest/replay invocation stopped
at `smac_lattice`; it was non-diagnostic ROS-domain/process interference, and
the isolated serial rerun passed without a source change. `AMR_CODEX_HANDOFF.md`
remains untouched.

### Prior handoff — 2026-09-25 planner-abort runtime diagnosis

The user resumed work and requested that Sol/high diagnose each failed runtime
before another source change. The AWS monitor/runner/diagnostics packet and
the navigation inflation packet are present in the dirty worktree and have
passed their focused source validation and affected-package builds.
`AMR_CODEX_HANDOFF.md` remains untouched.

The fresh runtime used `ROS_DOMAIN_ID=222` and run directory
`.ros_logs/aws_warehouse_runner_20260925_03/`. It stopped at the first
mandatory failure and exited 1. It reached two navigation goals, then the
third accepted mission goal ended at the planner boundary:

```text
Explorer: navigation goal ended with status 6;
Mission: global planning failed
Mission outcome: FAULT
Mission fault class: PLANNER_ABORT
Nav2: GridBased: failed to create plan, no valid path found.
```

The third candidate was Explorer cell `(246,17)`, world goal approximately
`(5.355706,-9.485328)`. The mission action was accepted at
`1790319701.548`, the compute-path action ended `ABORTED` at
`1790319703.570`, and no smoother or controller action was started. The
finalized bag is valid (one 671.5 MiB SQLite database, 142250 messages,
151.906 seconds); all owned processes exited after cleanup. The diagnostics
shutdown conversion error and Gazebo shutdown warnings occurred after this
primary fault and remain secondary.

Sol/high diagnosis: the 0.75 m inflation packet is not the defect. The exact
captured start, goal, map, raw global costmap, padded rectangular footprint,
lattice primitive file, and analytic-expansion-disabled replay reproduce
`SmacPlannerLattice createPath returned false` after 355000 iterations with
both `allow_unknown=true` and `allow_unknown=false`. Explorer's heading-aware
route search rejects raw lethal footprint overlap, but it does not model the
Smac lattice turning/cost-field reachability; therefore it can admit a
frontier that is footprint-clear yet has no valid Nav2 lattice route under the
conservative inflation field. This is safe unreachable-frontier evidence, not
a reason to weaken inflation, `allow_unknown`, planner collision checks, or
the smoother gate.

The remaining source-level recovery gap is narrow: Explorer currently recovers
only a mission terminal explicitly classified as `OBSTACLE_BLOCKAGE` and
faults every other non-success result. The mission supervisor intentionally
reports this exact planner-result boundary as `PLANNER_ABORT` with reason
`global planning failed`, while unavailable action servers and rejected goals
use different reasons but the same broad class. The next packet therefore
must recover only the exact `global planning failed` terminal, record the
destination under the current route-evidence fingerprint, wait for the
existing stationary/readiness proof, and replan. Unavailable/rejected
infrastructure, stale/malformed mission evidence, smoother failures, and
controller failures remain fail-closed faults. If no safe frontier remains,
the existing explained `INCOMPLETE` terminal is the accepted safe outcome.

Evidence is preserved in:

- `.ros_logs/aws_warehouse_runner_20260925_03/result.json`
- `.ros_logs/aws_warehouse_runner_20260925_03/launch.log`
- `.ros_logs/aws_warehouse_runner_20260925_03/events.jsonl`
- `.ros_logs/aws_warehouse_runner_20260925_03/evidence/exploration/`
- `/tmp/aws_run03_third_goal_snapshot.json`
- `/tmp/aws_run03_third_goal_replay_false.json`

The navigation packet remains implemented in
`src/amr_navigation/config/planner.yaml`,
`src/amr_mpc_controller/config/controller.yaml`, and
`src/amr_navigation/test/test_navigation_contract.py`: explicit `0.01 m`
padding, `0.75 m` global/local inflation, and a contract bound derived from
the padded footprint. Its focused source tests (`17 passed`), both package
CTests, both package builds, and scoped `git diff --check` passed before this
runtime. No AWS acceptance or terminal `COMPLETE` is claimed.

The Explorer recovery packet is now implemented only in
`src/amr_exploration/scripts/frontier_explorer.py` and its focused lifecycle
and contract tests. It recognizes a recoverable planner result only when the
accepted action is `ABORTED`, matching mission evidence is terminal `FAULT`,
the fault class is `PLANNER_ABORT`, `blockage_confirmed` is false, and the
reason is exactly `global planning failed`. It records the goal under the
current map/costmap fingerprint, releases motion ownership after terminal
proof, waits through the existing fresh-readiness/two-stationary-sample
recovery gate, and replans. Missing/stale/malformed evidence and all other
planner, smoother, controller, cancellation, or infrastructure failures still
use the fail-closed fault path.

Focused validation after the source change:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_exploration/test
192 passed, 2 xfailed, 1 xpassed

source /opt/ros/humble/setup.bash && source install/setup.bash && \
colcon build --packages-select amr_exploration --symlink-install \
  --event-handlers console_direct+
exit 0

ctest --test-dir build/amr_exploration --output-on-failure
3/3 passed

git diff --check -- \
  src/amr_exploration/scripts/frontier_explorer.py \
  src/amr_exploration/test/test_frontier_lifecycle.py \
  src/amr_exploration/test/test_frontier_contract.py
exit 0
```

Fresh runtime gate result: `.ros_logs/aws_warehouse_runner_20260925_04/`
used `ROS_DOMAIN_ID=221` and exited with `classification=INCOMPLETE`,
`pass=true`, and no first failure. It reached two goals, recorded two global
plans and two smoothed plans, then safely reported
`exploration incomplete: no safe costmap-valid frontier remains` with
`active=false`, `pending=false`, `cancel_owned_motion=false`,
`motion_stopped=true`, `fault_latched=false`, `goal_failures=0`, and
`unresolved_frontier_count=98`. Map saving and YAML/PGM verification passed;
the recorder finalized one valid 715.7 MiB bag with 150633 messages, and
cleanup left no owned-process survivors. The launch log contains no planner,
smoother, controller, or physical-contact failure. This is an operational
pass under the explicit Phase 15 safe-`INCOMPLETE` policy, not proof of the
stronger `COMPLETE` target or human map-quality acceptance.

The planner-abort recovery branch was not exercised by this run because route
selection reached safe no-frontier exhaustion before dispatching a third goal;
its exact status/reason/fault-class behavior is covered by the focused
Explorer tests. If work resumes toward the stronger `COMPLETE` target, first
define/authorize the exploration-policy change for the remaining unreachable
frontiers. Do not weaken inflation, unknown-space handling, planner/smoother
collision gates, or fail-closed fault handling. The unrelated broad CTest
world-size mismatch (`7950` actual versus `7954` expected) remains open and
must not be used as AWS acceptance evidence.

### Diagnostic continuation — final Run04 no reachable frontier (2026-09-25)

Sol/high continued from the safe Run04 `INCOMPLETE` result without changing
production behavior. The objective was to determine whether the stronger
`COMPLETE` target was being prevented by a stale pose, an over-conservative
Explorer route gate, or genuinely unreachable remaining frontiers.

The finalized Run04 bag was replayed read-only. The final map and raw global
costmap have identical `277x414`, `0.05 m`, unrotated geometry and the same
origin. The final map contains 98 raw frontier clusters. The selector returns
an empty set deterministically with the production footprint and
`require_path_clear=True` in about 2 seconds. The runtime trace independently
records three consecutive `route_search` spans with `candidate_count=0`
before terminal `INCOMPLETE`.

The final snapshot gate counts are:

```text
distance-eligible frontier cells       1959
endpoint cost < 253                     754
endpoint cost < 254                    1949
full-footprint-safe endpoint cost<253    12
clusters with any safe endpoint           1
full-footprint candidates without path   1  [(17,389)]
full-footprint candidates with path      0
```

The independent latest TF composition differs from the Explorer feedback
pose by only `0.0100 m`; using the feedback pose, composed TF pose, or zero
heading produces the same one endpoint without path proof and zero endpoints
with path proof. This falsifies stale-pose and heading-only explanations.

The exact safe endpoint `(17,389)` was sent through the installed Smac lattice
replay with the captured map, raw costmap, rectangular footprint, 0.5 m
turning-radius primitive set, analytic expansion disabled, and the final pose.
Both `allow_unknown=false` and `allow_unknown=true` returned
`SmacPlannerLattice createPath returned false` after 360000 iterations. A
diagnostic Smac 2D replay found a path, but its footprint safety check failed
with `worst_cost=255` at path index 198. Thus the 2D path is not an acceptable
alternative, and the configured lattice remains the authoritative planner.

This falsifies the hypothesis that Explorer is merely rejecting a valid safe
route. The strongest supported cause is that the only remaining
full-footprint-safe frontier region is unreachable by the configured planner;
the other raw frontiers have no safe endpoint under the current costmap.
`INCOMPLETE` is therefore the correct fail-closed result under the current
policy. No source files were edited in this diagnostic loop, no safety or
unknown-space gate was weakened, and no `COMPLETE` claim is made.

Read-only diagnostic commands/results:

```text
finalized Run04 bag selector replay: candidates []
runtime-trace extraction: route_search candidate_count=0 at map 66->76,
  79->88, and 91->99 (then terminal map versions 108-111)
standalone replay harness configure/build: /tmp/amr_replay_build.S4zi4t
Smac lattice allow_unknown=false: createPath false, 360000 iterations
Smac lattice allow_unknown=true:  createPath false, 360000 iterations
Smac 2D: planner_success=true, collision_free=false, worst_cost=255
```

**Safe resume point:** keep the current operational safe-`INCOMPLETE`
policy, or explicitly authorize a new exploration-completion policy packet.
Any new policy must define how unresolved raw frontiers are classified and
must preserve planner, footprint, inflation, unknown-space, smoother, and
fail-closed safety gates. Do not change production code merely to turn this
evidence into `COMPLETE`. The worktree remains the previously recorded dirty
worktree; this section is the only change from this diagnostic loop, and
`AMR_CODEX_HANDOFF.md` remains untouched.

### Session stop handoff — 2026-09-25

**Objective:** continue toward the existing AWS `COMPLETE` gate without
weakening safety or completion evidence.

**Diagnosis:** Run04's safe `INCOMPLETE` result is reproducible and justified.
The final evidence has 98 raw frontier clusters, but only 12
full-footprint-safe endpoint cells in one cluster; the sole selected endpoint
`(17,389)` is unreachable by the configured Smac lattice. Both unknown-space
settings fail the exact replay, while the 2D alternative has footprint cost
255 and is unsafe.

**Current result:** no implementation packet is justified. Production source,
planner configuration, unknown-space policy, and fail-closed terminal rules
are unchanged by this continuation. The only repository edit in this stop
record is this handoff file; `AMR_CODEX_HANDOFF.md` is untouched. The
worktree remains broadly dirty as listed by the pre-edit `git status --short`.

**Safe next step:** require explicit user authorization before defining a new
completion policy for unreachable raw frontiers. Sol/high must diagnose that
policy first; do not edit source or run another AWS attempt merely to convert
this evidence to `COMPLETE`. Preserve the footprint, inflation, unknown-space,
planner, smoother, controller, ownership, and fail-closed gates.

## Current checkpoint — 2026-09-23

### Latest checkpoint — lattice replay repair; AWS Run08 stopped at collision

This checkpoint supersedes the earlier failed Packet 6 replay and historical
diagnosis below. The user requested direct diagnosis/fixing without agents.
No independent review or full AWS acceptance is claimed.

#### Exploration acceptance assumption — revise later

The user observed in the GUI that some objects sit close enough to walls to
leave gaps the AMR cannot safely pass through. The AMR may nevertheless try to
enter such a gap and collide. This makes a 100% exploration target unrealistic
for this warehouse unless every area is physically reachable. Treat this as
the user's visual observation and explanation, not a proven sole cause for
every navigation failure.

The current AWS acceptance plan still uses the existing `COMPLETE` gate. The
user wants to revise the exploration plan and success criteria later; no
acceptance-policy change is made in this checkpoint. On resuming, first define
safe reachable-area completion and how blocked frontiers are reported. Keep
collision checks and controller safety behavior fail-closed; unreachable gaps
must not be treated as permission to force passage.

Confirmed replay defects: lattice backtrace theta is already radians, and
fractional map coordinates must be preserved using installed
`nav2_smac_planner::getWorldCoords`. The corrected harness also matches the
downstream collision gate: footprint cost 253 is permitted, 254/255 rejected,
including when planner `allow_unknown` is true. The recorded unsafe path is
a negative control, not a requirement that the candidate must make it safe.

The remaining lattice collision was traced with GDB: analytic shortcut poses
are checked at truncated map coordinates, but published at fractional cell
positions. At raw pose `(181.237671, 92.8779907, 0)`, the actual footprint
cost was 254 versus 253 at the checked truncated position. Disabling analytic
expansion before planner construction removed lethal overlap on the captured
route. This is a demonstrated route-specific mitigation, not a claim that
all upstream lattice collision behavior is repaired.

Implemented files:

- `src/amr_navigation/test/replay/`: corrected conversion and collision gate,
  analytic expansion maximum length 0, explicit Run07 regression mode, and
  C++ conversion/footprint contract tests.
- `src/amr_navigation/config/planner.yaml`: SmacPlannerLattice, analytic
  expansion disabled, internal smoothing disabled; external collision-checked
  SimpleSmoother retained. Footprint, inflation and controller safety unchanged.
- `src/amr_navigation/launch/amr_navigation.launch.py`: installed 5cm/0.5m
  differential-drive lattice primitive file passed to planner.
- `src/amr_navigation/package.xml`: explicit `nav2_smac_planner` dependency.
- `src/amr_navigation/test/test_navigation_contract.py`: migration assertions.

Fresh validation (all commands exited 0):

```text
cmake --build /tmp/amr-planner-replay-build.x00Wyd --parallel 2
ctest --test-dir /tmp/amr-planner-replay-build.x00Wyd -R '^planner_replay_contract$' --output-on-failure
  1/1 passed
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_navigation/test/test_navigation_contract.py
  5 passed
colcon build --packages-select amr_navigation --symlink-install --event-handlers console_direct+
git diff --check -- src/amr_navigation
```

Replay commands, from workspace root (ROS_DOMAIN_ID=0):

```text
/tmp/amr-planner-replay-build.x00Wyd/planner_replay_lattice --snapshot .ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_snapshot.json --lattice /opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json --planner lattice --run-smoother --output .ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_lattice_no_shortcut.json
python3 src/amr_navigation/test/replay/run_planner_replay.py --snapshot .ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_snapshot.json --backend-2d /tmp/amr-planner-replay-build.x00Wyd/planner_replay_2d --backend-lattice /tmp/amr-planner-replay-build.x00Wyd/planner_replay_lattice --lattice /opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json --report .ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_regression.json --run07-regression
```

Both exited 0. Lattice produced 167 poses in 175911 iterations, worst
footprint cost 253, passing its smoother and endpoint/yaw checks. Six
regression cases passed: unsafe 2D negative control, lattice candidate,
blocked start/goal, and forbidden-unknown start/goal. Recorded unsafe path
still fails at lethal cost 254 as expected. Endpoint compatibility is not
dynamic controller proof. Separate narrow-turn/shelf fixtures and full runtime
coverage remain unverified; test totals are not AWS acceptance.

Run08 runtime result:

- Run directory: `.ros_logs/aws_warehouse_runner_20260923_08/`.
- ROS domain 231; unique runner-owned partition; Gazebo GUI and RViz enabled.
- Started with `source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /tmp/amr_aws_run08.py`.
- Launcher reuses Run07's exact command arrays, replacing only evidence/run
  paths. Durable command authority is Run08 `run.json` and `processes.json`.
- The run stopped at the first mandatory fault: at ROS time
  `1790180767.995482766`, Regulated Pure Pursuit reported `detected collision
  ahead`; the mission supervisor then reported `Mission aborted: path
  following failed`. Two goals had been reached earlier, but the run did not
  reach terminal `COMPLETE`.
- `result.json` classifies the run `FAULT` (`pass=false`). Runner cleanup
  escalated to SIGTERM, reports no surviving owned processes, and the
  recorder finalized successfully. Post-run `ros2 bag info` exited 0: one
  database, metadata present, 78.8 MiB, 157.62 seconds, 61,540 messages.
- Full logs and bag: `.ros_logs/aws_warehouse_runner_20260923_08/`.
  The precise obstacle/gap contribution has not been extracted from the bag;
  correlate the failed local plan, footprint, and map before claiming a
  source-level root cause.

Run08 is terminal; its live bag is finalized and may be inspected. The user
asked to defer changing the exploration plan. Resume with that planning
discussion before another acceptance run. Preserve all unrelated dirty work
and never touch `AMR_CODEX_HANDOFF.md`.

Packets 1–5 are implemented in the current worktree. The AWS runner now owns
the recorder, monitor, diagnostics, and launch in one session; validates unique
run identity and ROS domains 0–232; preserves first-failure evidence; forbids
active SQLite bag inspection; and verifies bounded cleanup. The Explorer
changes repair lifecycle fixtures, reject unswept initial turns, separate
costmap content from `metadata.update_time`, and provide opt-in runtime
diagnostics. The diagnostics node also now accepts Humble's byte-valued
`DiagnosticStatus.level`.

Focused validation of the maintained runtime components passed:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test/test_aws_exploration_monitor.py \
  src/amr_simulation/test/test_aws_exploration_runner.py
15 passed
```

`colcon build --packages-select amr_simulation --symlink-install
--event-handlers console_direct+` exited 0. A broader scoped test run still
has one unrelated pre-existing world-size assertion (`7954` expected versus
`7950` actual) in `test_aws_warehouse_exploration.py`; it is not a runtime
acceptance result.

### Fresh runtime evidence

The maintained runner was exercised on valid domains 224–227 and stopped at
the first mandatory failure in each attempt. Run 01 exposed the diagnostics
byte-level deserialization defect above and was not otherwise diagnostic. Run
02 crossed the observer/startup boundary, completed two navigation goals, then
hit the first real path-following fault: Regulated Pure Pursuit reported a
collision ahead. Its finalized bag is at
`.ros_logs/aws_warehouse_runner_20260921_02/evidence/exploration/`.
Run 03 stopped at the portable stow startup fault and is non-diagnostic.

Run 04 successfully captured local and global costmaps, then stopped at the
first smoother failure:

```text
Smoothed path leads to a collision at x: 4.257703, y: -9.621695,
theta: 0.257624
Mission aborted: path smoothing failed or was incomplete
```

The finalized Run 04 bag is at
`.ros_logs/aws_warehouse_runner_20260921_04/evidence/exploration/` and has
valid metadata, 39,194 messages, and no surviving run-owned processes. Replay
of the latest recorded global raw costmap at that pose found a free center
cell, but the rectangular footprint overlapped 116 cells with raw cost 253 and
2 cells with raw cost 254. This confirms a safety-gate mismatch between the
Explorer's footprint admission and the existing Nav2 smoother; it does not
authorize weakening the smoother, inflation, or collision thresholds. No
planner YAML, launch, or dependency change was made from this runtime
failure, and no `COMPLETE` acceptance is claimed.

### GUI runtime reconfirmation — `aws_warehouse_runner_20260922_02`

A fresh GUI run used `ROS_DOMAIN_ID=229` with
`headless:=false`, `software_rendering:=auto`, and `rviz:=true`. Gazebo and
RViz both started successfully; RViz reported OpenGL 4.6. The robot then
remained stationary because Explorer never accepted a navigation goal. Live
diagnostics repeatedly reported:

```text
frontier plan discarded because refreshed TF is stale
```

The live state remained `SCANNING` with `active=false`, `pending=false`, and
`motion_generation=0`. The finalized bag contains zero `/amr/plan`,
`/amr/plan_smoothed`, and navigation action-status messages, confirming that
the GUI was not the cause. The run was intentionally interrupted after this
boundary was confirmed; `result.json` records `INTERRUPTION`, bounded cleanup,
and no surviving processes. The finalized bag is 111.6 MiB, 212.9 seconds,
and 92,247 messages at
`.ros_logs/aws_warehouse_runner_20260922_02/evidence/exploration/`.

This reconfirmed the Packet 5 TF-admission symptom, but did not by itself
identify the private buffer boundary. The next diagnostic run measured that
boundary and is recorded below; no freshness-timeout or planner/configuration
change was made from the GUI run alone.

### AWS TF listener repair and runtime result — 2026-09-23

The diagnostic GUI run `aws_warehouse_runner_20260923_01` (ROS domain 228,
`headless:=false`, `software_rendering:=auto`, `rviz:=true`) measured the
private TF boundary. Upstream TF was current, but the Explorer's in-process
buffer returned samples about 1.41–1.48 seconds old after the Python route
search. The final evidence check took about 0.14–0.17 seconds, so the existing
1.5-second freshness gate correctly rejected the sample as
`frontier plan discarded because refreshed TF is stale`. The run was stopped at
that first confirmed boundary and finalized as `INTERRUPTION` with no surviving
processes.

The scoped repair in `src/amr_exploration/scripts/frontier_explorer.py` gives
TF its own listener node and `SingleThreadedExecutor` thread, with explicit
shutdown in `destroy_node()`. It does not increase TF freshness or future
tolerances and keeps the final reservation proof fail-closed.

Focused validation passed:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_exploration/test/test_frontier_lifecycle.py \
  src/amr_simulation/test/test_aws_exploration_monitor.py \
  src/amr_simulation/test/test_aws_exploration_runner.py
139 passed, 2 xfailed, 1 xpassed
```

An independent construction/cleanup probe also confirmed the listener uses a
separate executor and its thread is stopped by `destroy_node()`.

The fresh GUI verification run `aws_warehouse_runner_20260923_02` (ROS domain
230) reached motion and crossed the repaired boundary: it recorded 3
reservations, 2 successful navigation goals, 3 plans, and 3 smoothed paths.
Runtime traces contained 194 accepted lookups, 4 accepted refreshes with
0.01–0.06 seconds of TF age, and no stale-TF rejection. It then stopped at the
first mandatory downstream failure:

```text
Smoothed path leads to a collision at x: -4.281184, y: 8.342126,
theta: 2.102695
Mission aborted: path smoothing failed or was incomplete
```

The runner classified this as `FAULT`, finalized a valid 40.9 MiB bag with
33,999 messages, and confirmed bounded cleanup with no survivors. No AWS
acceptance or terminal Explorer verdict is claimed. Analyzer policy is now
`gpt-5.6-sol` with `high` reasoning in `/home/pete/.codex/config.toml`.

### Packet 6 replacement artifact and replay result — 2026-09-23

The preserved Run63 snapshots were unavailable, so the user's explicit
instruction to rerun the AWS simulation authorized a replacement artifact.
Run `aws_warehouse_runner_20260923_07` used the GUI command with
`headless:=false`, `software_rendering:=auto`, `rviz:=true`, and
`auto_start_exploration:=true` on `ROS_DOMAIN_ID=230`. It reached 3
reservations, published 3 plans and 3 smoothed paths, and then stopped at the
first mandatory failure:

```text
Smoothed path leads to a collision at x: 2.370111, y: -5.684254,
theta: -0.507599
Mission aborted: path smoothing failed or was incomplete
```

The run was classified `FAULT`, finalized a valid bag at
`.ros_logs/aws_warehouse_runner_20260923_07/evidence/exploration/`, and
confirmed bounded cleanup with no surviving run-owned processes. The finalized
bag has about 44.6 MiB, 100.08 seconds, and 37,507 messages.

The replacement planner snapshot is
`.ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_snapshot.json`.
It contains the latest 137-pose plan, the preceding map and raw global
costmap, and the observed rectangular footprint
`((0.61,0.41),(0.61,-0.41),(-0.61,-0.41),(-0.61,0.41))`.
It was extracted from the finalized bag with:

```text
source /opt/ros/humble/setup.bash && source install/setup.bash &&
python3 src/amr_navigation/scripts/extract_planner_replay_snapshot.py \
  --bag-dir .ros_logs/aws_warehouse_runner_20260923_07/evidence/exploration \
  --output .ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_snapshot.json
```

The standalone replay harness is under
`src/amr_navigation/test/replay/`, with the finalized snapshot extractor at
`src/amr_navigation/scripts/extract_planner_replay_snapshot.py`. It builds
successfully with the installed Humble Smac 2D, Smac lattice, and smoother
libraries. The default gate runner stopped at the first mandatory failure with
exit 2; an explicitly diagnostic continuation recorded both backends:

```text
cmake --build /tmp/amr-planner-replay-build.x00Wyd --parallel 2  # exit 0
```

```text
SmacPlanner2D: planner_success=true, iterations=11715,
  endpoint_compatible=true, rectangular_collision_free=false,
  first_collision_index=102, worst_cost=254,
  candidate_smoother.success=false
SmacPlannerLattice: planner_success=true, iterations=2561,
  endpoint_compatible=true, controller_compatible=true,
  rectangular_collision_free=false, first_collision_index=133,
  worst_cost=254, candidate_smoother.success=false
Recorded Run07 path through SimpleSmoother:
  smoother_completed=true, rectangular_collision_free=false,
  first_collision_index=101, worst_cost=254
```

Evidence is preserved in
`.ros_logs/aws_warehouse_runner_20260923_07/evidence/planner_replay_gate.json`
and `planner_replay_gate_all.json`. The replay gate therefore fails for both
planner candidates; blocked endpoint, unknown-space, narrow-turn, and shelf
fixture cases were not run after the mandatory baseline failure. No planner
YAML, launch, dependency, or contract migration is authorized from this
evidence. Safe resume point: Sol/high diagnosis of the rectangular footprint
path mismatch before any production planner edit.

## Historical diagnosis — superseded where noted by latest checkpoint

Run64 used one recorder and one headless AWS launch with `ROS_DOMAIN_ID=182`
and `GZ_PARTITION=amr_aws_warehouse_20260921_64`. It reached two goals, then
repeatedly discarded plans because refreshed TF was stale or unavailable. A
preserved snapshot admits frontier `(59, 356)`, but Python route selection
took 3.31 s cold and 3.17 s warm; heading-aware route search dominates the
profile. Upstream recorded TF was current at the published discard times, so
the exact private lookup/receipt failure is still unproven. Do not increase a
freshness timeout or add executor threads until that boundary is measured.

The recorder exception was misattributed. Run64's recorder log ended with:

```text
rosbag2_storage_plugins::SqliteException
SQLite error (5): database is locked
```

The log was last written about 163 seconds after topic discovery. An isolated
rosbag2 probe reproduced the same error only when a read cursor was held on
the active SQLite file; closing the reader restored writes. Therefore never
open an active bag with SQLite, `rosbag2_py`, `ros2 bag info`, or an integrity
query. Finalize the recorder before any inspection. Run64 is not acceptance
evidence because it lacks a terminal result and finalized metadata.

Host inspection also found Run63 still alive: its wrapper was waiting while
Gazebo, Explorer, navigation, and adapter descendants remained in process
group `713820`. The old wrapper does not reliably terminate descendants.

The current `SmacPlanner2D` change is not footprint-equivalent to the custom
route check. Installed SmacPlanner2D configures `GridCollisionChecker` with
radius mode; it does not prove the rectangular footprint used by the
Explorer/smoother. The navigation package also lacks an explicit
`nav2_smac_planner` dependency. Treat the existing YAML change as unaccepted
until replay and runtime evidence prove planner/smoother compatibility.

The route search has a confirmed safety gap: first-step validation checks the
current and movement orientations but skips the swept turn between them. A
fixture with a lethal cell at an intermediate heading admitted a route. Four
existing lifecycle tests currently fail because their fixtures place the robot
outside the tiny costmap; a separate test helper can generate forbidden ROS
domains 233–239.

## Luna/max implementation plan

Run packets sequentially. Before each packet, Luna must run `git status
--short`, reread the exact targets, preserve unrelated dirty work, inspect the
complete scoped diff, and run focused validation. If evidence contradicts a
packet, Luna stops and returns to Sol/xhigh.

### Packet 1 — Make AWS runtime ownership and evidence trustworthy

Allowed paths: a maintained AWS runner/monitor under
`src/amr_simulation/scripts/`, their existing/new tests,
`src/amr_simulation/CMakeLists.txt`, and `docs/SIMULATION_COMMANDS.md`.

The runner must validate a unique run directory and domain 0–232; start
recorder, monitor, diagnostics, and launch under one owned process session;
record child process groups and exit identities; preserve the first failure;
classify `COMPLETE`, `INCOMPLETE`, `FAULT`, recorder failure, observer failure,
interruption, and launch exit separately; detect smoothing/path-following
abort logs; and bound graceful shutdown/escalation for every descendant.
Only `COMPLETE` with no active/pending/cancel-owned motion can be a pass.
Never inspect the active SQLite bag. Finalize and verify all run-owned
processes have exited before post-run bag checks.

Tests must cover terminal classification, recorder crash, monitor exit,
surviving descendants, cleanup escalation, invalid/reused run identity, and
attempted live-bag inspection. Prediction: a stopped run leaves no owned
processes and a recorder failure cannot be mistaken for acceptance.

### Packet 2 — Repair lifecycle regression fixtures and domain bounds

Allowed path: `src/amr_exploration/test/test_frontier_lifecycle.py`.
Repair only the four failing fixtures so robot pose, footprint, and expected
route fit valid geometry; retain negative out-of-map rejection cases; and
constrain all generated test domains to 0–232. No production behavior changes.

### Packet 3 — Validate the initial swept turn

Allowed paths: `src/amr_exploration/scripts/frontier_algorithm.py` and its
existing test file. Pass the actual starting yaw into first-step feasibility;
sample the shortest wrapped turn to each first movement heading with the
existing exact footprint check; and apply the same proof in refreshed
route-start validation. Preserve center/diagonal/endpoint rejection at 253,
footprint rejection at 254/255, and footprint-only permission for 253.
Regression: the constrained intermediate-heading collision must be rejected,
while clear turns and Run03's 253 case remain admissible.

### Packet 4 — Separate costmap content from update time

Allowed paths: `frontier_explorer.py` and its lifecycle test. Exclude only
`metadata.update_time` from content equality. Keep frame, geometry, layer,
creation identity, cell data, receipt freshness, and ROS timestamp checks.
Test identical fresh data with a newer update time plus geometry/data/freshness
changes. This is a confirmed latent defect, not the proven cause of Run64.

### Packet 5 — Instrument the TF admission boundary

Allowed paths: `frontier_explorer.py`, its lifecycle test, and the maintained
diagnostic monitor. Add diagnostics disabled by default for generation, wall
and ROS times before/after clustering, route search, lookup, refresh, action
wait, and reservation; returned TF stamps and validation ages; lookup errors;
map/costmap versions; evidence-change reasons; authority receipt ages; and
reservation duration. Run one clean AWS attempt using Packet 1 and stop at
the first failure. Classify whether lookup returns old data, lookup throws,
the sample ages during measured work, or evidence/authority changes. No
timeout increase, executor change, or retimestamping is allowed from this
packet alone.

### Packet 6 — Prove a planner with the required footprint behavior

First add an offline replay harness/test for preserved Run63 map, costmap,
start, and goal snapshots. Compare installed SmacPlanner2D with the
installed SmacPlannerLattice differential-drive primitive set, checking the
actual rectangular footprint and then the existing collision-checked
smoother. Include blocked start/goal, unknown space, narrow turns, reachable
shelf routes, final orientation, and controller compatibility. Do not relax
inflation, collision checks, or controller safety limits.

Only after replay passes may Luna change the navigation YAML/launch/package
dependency and contract test. If the lattice candidate fails replay or
controller compatibility, stop and return evidence; do not substitute another
planner speculatively.

## Validation and runtime acceptance

Use focused tests first, then affected-package symlink builds and scoped
`git diff --check`. Aggregate totals are not runtime proof. Before AWS, clean
only the verified old Run63 process group, confirm the TF overlay and startup
wrappers resolve from the workspace, and choose an unused domain/partition.

Final acceptance requires one uninterrupted AWS run reaching `COMPLETE`,
continued mapping through reachable shelves, no authority/freshness/terminal
fault, nonzero finalized raw-costmap/map/TF/exploration/navigation rows,
valid bag metadata/database, and no surviving run-owned processes. Stop at the
first mandatory failure, preserve evidence, and return to Sol/xhigh before any
new edit or run. Independent Sol/xhigh source review follows the runtime
attempt.

## Worktree rules and records

Preserve all unrelated dirty files, `phase14_evidence/`, vendor/runtime files,
and `SESSION_HANDOFF_HISTORY.md`. Keep this file latest-only; put detailed
historical records in the history or a separate AWS diagnosis document. Do
not reset, stage, commit, push, rewrite history, install dependencies, alter
system configuration, or modify `AMR_CODEX_HANDOFF.md` without explicit
authorization. Explorer navigation simplification remains a separate design
packet after runtime acceptance.

## Historical stop handoff — 2026-09-25 (Asia/Bangkok), superseded by Run05

**Model policy:** Sol/high is the analyzer and independent reviewer; Luna/max is the only implementation writer. Astra was not used. The user explicitly requested a stop after the next implementation packet was prepared.

**Completed before this stop**

- Run08 safety mechanism was reconstructed from the finalized bag and replay evidence. The controlled GUI run stopped at the confirmed smoothed-path collision boundary; the evidence did not prove physical contact. Observation-only simulation diagnostics were added and the simulation command was updated in this workspace and `/home/pete/sh&text/amr_cmd`.
- The planner review was independently rerun: Smac lattice replay with `allow_unknown=false` and the external smoother produced `success=true`, `collision_free=true`, `smoother_completed=true`, and `controller_compatible=true`.
- The mission supervisor now publishes UUID-correlated `/amr/mission/status` diagnostics with explicit stage/outcome/reason/blockage/fault fields. Mission/workspace contracts passed (`11 passed`); the `amr_mission` build and registered tests passed.
- Luna/max implemented the Explorer recovery packet in `frontier_explorer.py` and its focused tests. Sol/high independently reran lifecycle/contract tests (`136 passed, 2 xfailed, 1 xpassed`), frontier algorithm tests (`53 passed`), `colcon build --packages-select amr_exploration` (pass), `py_compile` (pass), and scoped `git diff --check` (pass). The packet adds UUID-correlated mission evidence, fail-closed generic navigation results, `RECOVERY_WAIT`, fresh readiness plus two-sample stationary TF proof, bounded three-attempt evidence-keyed blockage deferral, and terminal progress/motion fields.

**Packet pending at the historical stop; subsequently completed**

Sol/high diagnosed the next packet but it was interrupted before a Luna handoff result at that historical point. The allowed target was the AWS monitor/runner/diagnostics path: require `motion_stopped` and terminal counts/mission fields, accept explained safe `INCOMPLETE` as an operational pass, defer abort-log classification until structured terminal evidence, add pre-shutdown Nav2 map saving into the unique run evidence directory with YAML/image verification, record `/amr/mission/status` in the bag topic set, and add focused tests. The packet was subsequently implemented, source-validated, and exercised by Run05; see the current Safe Reachable-Area Completion and Runtime acceptance records at the top of this file.

**Historical safe resume point — superseded**

This resume instruction is superseded. The maintained monitor/runner and diagnostics path is present, validated, and covered by the fresh Run05 simulation acceptance above. Do not resend that packet or launch a redundant runtime without a new user-authorized requirement. Do not touch `AMR_CODEX_HANDOFF.md`; preserve all existing unrelated work.

**Luna implementation ledger:** `LUNA_IMPLEMENTATION_LEDGER.md` records 7 prior mistakes (each with date/time/topic/reason/likely cause). No new mistake was recorded after the raster-fixture diagnosis because no post-diagnosis implementation result was available.
