# Approved Gazebo stop receipt

User explicitly approved Stop only these three servers, then stop all of them.
Scope ONLY106994,113056,128495; no other process/system/configuration changes.
Revalidated2026-10-05 13:34:09UTC: all three still original command
gz sim -r -s -v 2 /home/pete/amr_ws/src/amr_simulation/worlds/aws_warehouse.sdf,
parent1910 user-systemd. Start times Bangkok15:15:48,15:29:05,19:14:52 unchanged.
Plan: SIGINT to exactly these PIDs; verify exit before strict host admission.
Other user processes, logs and artifacts untouched. Results pending below.

Executed kill -INT -- 106994 113056 128495, exit0. Follow-up ps reports no
remaining entry for any of the three PIDs; broader pgrep only matched its own
inspection shell, no Gazebo server. Graceful exit confirmed; no escalation used.
